"""Text and binary protocol parsing for the tested STM32 IRPHOTO4 firmware.

How the firmware CLI talks (see cli_service.c) and what this module relies on:

* Every byte the host sends is echoed back by the firmware. Characters of a
  ``seal`` / ``unseal`` / ``erase`` password are echoed as ``*``.
* A command ends with the unterminated prompt ``IR> ``.
* The firmware can print *unsolicited* notices at any time, e.g.
  ``[CONFIG] persistent save complete; new generation active.``. Each notice is
  followed by its own extra ``IR> `` prompt. If the host simply reads "until the
  next prompt", that extra prompt is taken for the end of the *next* command and
  every later answer is shifted by one. The parser therefore (a) drops stale
  bytes before sending a command and (b) recognises the answer to a command by
  the echo of that command.
"""
from __future__ import annotations
import re, time
from dataclasses import dataclass
from .errors import ConnectionLost, DeviceCommandError, ProtocolError, ReadTimeout, TransportError

ANSI_RE = re.compile(rb"\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1b\\))")
# Firmware redraws its input line as "\r ESC[2K": whatever was printed on that line before is erased.
ERASED_LINE_RE = re.compile(rb"[^\r\n]*\r\x1b\[2K")
PROMPT = b"IR> "
CONFIRM_PROMPT = b"Type YES to continue or NO to cancel: "
ACK, NACK, CANCEL = b"\x06", b"\x15", b"\x03"
MAX_COMMAND_LENGTH = 79  # firmware CLI_LINE_SIZE is 80 including the terminator

# Command lines that the firmware echoes while a machine protocol is starting.
_ECHO_LINES = ("photos list", "photo all", "journal all")


def strip_ansi(data: bytes | str) -> str:
    raw = data.encode("utf-8", errors="replace") if isinstance(data, str) else data
    raw = ERASED_LINE_RE.sub(b"", raw)
    return ANSI_RE.sub(b"", raw).decode("utf-8", errors="replace").replace("\x00", "")


def clean_transcript(data: bytes, command: str | None = None) -> str:
    text = strip_ansi(data)
    text = text.replace("\r", "")
    lines = text.split("\n")
    if command and lines and lines[0].strip() == command.strip():
        lines = lines[1:]
    joined = "\n".join(lines)
    joined = re.sub(r"(?:^|\n)IR>\s*$", "", joined)
    return joined.strip("\n ")


def clean_notice(data: bytes | str) -> str:
    """Printable form of unsolicited device text: no ANSI, no CR, no prompts, no blank lines."""
    text = strip_ansi(data).replace("\r", "").replace("IR> ", "")
    lines = [line.rstrip() for line in text.split("\n")]
    return "\n".join(line for line in lines if line.strip()).strip()


def is_cli_sync_response(data: bytes) -> bool:
    """Return true only for a response to the harmless NO/prompt sync handshake."""
    text = strip_ansi(data).replace("\r", "")
    lines = text.split("\n")
    return ("NO" in lines or "IRPHOTO_ABORT," in text or "IRPHOTOS_ABORT," in text
            or "IRPHOTO_ALL_ABORT," in text or "Destructive operation cancelled" in text)


def _is_password_char(line: str) -> bool:
    """Port of Cli_IsPasswordCharacter(): is the LAST char of ``line`` echoed as '*'?"""
    n = len(line)
    low = line.lower()
    for prefix in ("seal ", "unseal "):
        if n > len(prefix) and low.startswith(prefix):
            return True
    if n < 6 or not low.startswith("erase "):
        return False
    token_end = 6
    while token_end < n and not line[token_end].isspace():
        token_end += 1
    if token_end == n:
        return False
    index = token_end
    while index < n and line[index].isspace():
        index += 1
    return n > index


def masked_echo(command: str) -> str:
    """The text the firmware echoes back for ``command`` (passwords become '*')."""
    return "".join("*" if _is_password_char(command[:i + 1]) else c for i, c in enumerate(command))


def check_command_text(command: str) -> None:
    if not command.strip():
        raise ValueError("empty command")
    if any(not 0x20 <= ord(c) <= 0x7E for c in command):
        raise ValueError("device commands may contain printable ASCII characters only")
    if len(command) > MAX_COMMAND_LENGTH:
        raise ValueError(f"command is too long (device limit is {MAX_COMMAND_LENGTH} characters)")


@dataclass(frozen=True)
class PhotoHeader:
    event_id: int; width: int; height: int; pixel_format: int; bytes_per_pixel: int
    total_bytes: int; crc32: int; chunk_size: int
    @classmethod
    def parse(cls, line: str) -> "PhotoHeader":
        f=line.strip().split(",")
        if len(f)!=9 or f[0]!="IRPHOTO4": raise ProtocolError(f"invalid IRPHOTO4 header: {line!r}")
        try: obj=cls(int(f[1]),int(f[2]),int(f[3]),int(f[4]),int(f[5]),int(f[6]),int(f[7],16),int(f[8]))
        except ValueError as exc: raise ProtocolError(f"invalid numeric field in photo header: {line}") from exc
        if min(obj.event_id,obj.width,obj.height,obj.bytes_per_pixel,obj.total_bytes,obj.chunk_size)<=0:
            raise ProtocolError("photo header contains non-positive values")
        if obj.width*obj.height*obj.bytes_per_pixel != obj.total_bytes:
            raise ProtocolError("photo header geometry does not match payload size")
        if obj.total_bytes > 8*1024*1024 or obj.chunk_size>4096 or obj.crc32>0xFFFFFFFF:
            raise ProtocolError("photo header exceeds safety limits")
        if obj.pixel_format != 1 or obj.bytes_per_pixel != 2:
            raise ProtocolError(f"unsupported pixel format={obj.pixel_format}, bpp={obj.bytes_per_pixel}")
        return obj

@dataclass(frozen=True)
class PhotoListItem:
    event_id:int; tick_ms:int; width:int; height:int; pixel_format:int; bpp:int; size:int; crc32:int
    @classmethod
    def parse(cls,line:str)->"PhotoListItem":
        f=line.strip().split(",")
        if len(f)!=9 or f[0]!="IRPHOTOS_ITEM": raise ProtocolError(f"invalid photo list item: {line!r}")
        try: x=cls(int(f[1]),int(f[2]),int(f[3]),int(f[4]),int(f[5]),int(f[6]),int(f[7]),int(f[8],16))
        except ValueError as exc: raise ProtocolError(f"invalid photo list number: {line}") from exc
        if min(x.event_id,x.width,x.height,x.bpp,x.size)<=0 or x.tick_ms<0 or x.size!=x.width*x.height*x.bpp:
            raise ProtocolError("photo list item has inconsistent geometry")
        if x.pixel_format!=1 or x.bpp!=2 or x.crc32>0xFFFFFFFF or x.size>8*1024*1024:
            raise ProtocolError("photo list item uses an unsupported pixel format or exceeds storage limits")
        return x

@dataclass(frozen=True)
class JournalItem:
    event_id: int; type: str; mode: str; tick_ms: int; value: int; threshold: int
    photo_outcome: int; photo_reason: int
    @classmethod
    def parse(cls, line: str) -> "JournalItem":
        f = line.strip().split(",")
        if len(f) != 9 or f[0] != "IRJOURNAL_ITEM": raise ProtocolError(f"invalid journal item: {line!r}")
        try:
            return cls(int(f[1]), f[2], f[3], int(f[4]), int(f[5]), int(f[6]), int(f[7]), int(f[8]))
        except ValueError as exc:
            raise ProtocolError(f"invalid journal number: {line}") from exc


class ProtocolParser:
    """Small parser; binary payload is never passed through text cleanup."""
    def __init__(self, transport, command_timeout: float=5.0, on_text=None):
        self.transport=transport; self.command_timeout=command_timeout; self.on_text=on_text or (lambda _s:None)

    # ------------------------------------------------------------------ notices
    def flush_async(self, show: bool = True) -> str:
        """Take everything that is buffered right now (stale prompts, unsolicited
        notices) and optionally print the meaningful part. Returns the cleaned text."""
        drain = getattr(self.transport, "drain", None)
        if drain is None:
            return ""
        text = clean_notice(drain())
        if text and show:
            self.on_text(text)
        return text

    def wait_async(self, markers: tuple[str, ...], timeout: float) -> str | None:
        """Wait for an unsolicited notice that contains one of ``markers``
        (case-insensitive). Other notices seen meanwhile are printed.
        Returns the matching notice text, or None on timeout."""
        deadline = time.monotonic() + timeout
        wanted = tuple(m.lower() for m in markers)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                raw = self.transport.read_until(PROMPT, remaining)
            except ReadTimeout as exc:
                if exc.partial:
                    self.transport.unread(exc.partial)
                return None
            text = clean_notice(raw)
            if not text:
                continue
            if any(m in text.lower() for m in wanted):
                return text
            self.on_text(text)

    # --------------------------------------------------------- sync / recovery
    def sync(self, attempts: int = 3, timeout: float = 5.0) -> bool:
        """Bring the CLI to a known idle state and consume everything pending.

        Ctrl+C aborts a stranded photo/list stream, ``NO`` cancels a pending erase
        confirmation and is a harmless unknown command in the normal CLI."""
        for _ in range(attempts):
            drain = getattr(self.transport, "drain", None)
            if drain: drain()
            self.transport.write(CANCEL)
            self.transport.send_command("NO")
            for _wait in range(3):
                try:
                    response = self.transport.read_until(PROMPT, timeout)
                except TransportError as exc:
                    if isinstance(exc, ConnectionLost):
                        raise
                    break
                if is_cli_sync_response(response):
                    # The firmware may still emit late bytes (e.g. the answer to the
                    # NO that was dropped/handled after an abort). Let them arrive and
                    # discard them silently.
                    time.sleep(0.25)
                    if drain: drain()
                    return True
        return False

    # ----------------------------------------------------------------- commands
    def exchange(self, command: str, timeout: float | None = None, echo: str | None = None,
                 extra_markers: tuple[bytes, ...] = ()) -> tuple[str, bytes]:
        """Send one command line and return (response_text, marker).

        ``marker`` is PROMPT or one of ``extra_markers``. The response is
        everything after the echo of the command; text before the echo is an
        unsolicited notice and is printed through ``on_text``."""
        check_command_text(command)
        self.flush_async()
        self.transport.send_command(command)
        return self._read_response(masked_echo(command) if echo is None else echo,
                                   timeout or self.command_timeout, extra_markers)

    def _read_response(self, echo: str, timeout: float, extra_markers: tuple[bytes, ...]) -> tuple[str, bytes]:
        deadline = time.monotonic() + timeout
        markers = (PROMPT,) + tuple(extra_markers)
        want = echo.strip()
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ReadTimeout("timeout waiting for the device response")
            raw, marker = self.transport.read_until_any(markers, remaining)
            text = strip_ansi(raw).replace("\r", "")
            text = text[:-len(marker)] if text.endswith(marker.decode("ascii")) else text
            lines = text.split("\n")
            index = next((i for i, line in enumerate(lines) if line.replace("IR> ", "").strip() == want), -1)
            if index < 0:
                # No echo of our command: this is an unsolicited notice (with its own
                # prompt). Show it and keep waiting for the real answer.
                notice = clean_notice(raw)
                if notice:
                    self.on_text(notice)
                continue
            before = clean_notice("\n".join(lines[:index]))
            if before:
                self.on_text(before)
            body = "\n".join(lines[index + 1:]).strip("\n").rstrip()
            return body, marker

    def command(self, command:str, timeout:float|None=None, allow_device_error:bool=False, echo:str|None=None)->str:
        text, _marker = self.exchange(command, timeout=timeout, echo=echo)
        if not allow_device_error and re.search(r"(?:^|\n)ERR:",text): raise DeviceCommandError(text)
        return text

    def wait_prompt(self, timeout:float=5.0)->str:
        raw=self.transport.read_until(PROMPT,timeout)
        text=clean_transcript(raw)
        if text: self.on_text(text)
        return text

    def next_text_line(self, timeout:float=5.0)->str:
        return strip_ansi(self.transport.read_line(timeout))

    def next_record(self, prefixes:tuple[str,...], timeout:float, show_unmatched:bool=True)->str:
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            line=self.next_text_line(max(0.05,deadline-time.monotonic()))
            if line.startswith(prefixes): return line
            # Firmware echoes commands even in machine-protocol mode; hide only
            # the known command echo, while forwarding unrelated async text.
            lower=line.strip().lower()
            if lower in _ECHO_LINES or lower.startswith("photo id "):
                continue
            if line and show_unmatched: self.on_text(line)
        raise ReadTimeout(f"timeout waiting for protocol record {prefixes}")

    # ------------------------------------------------------- machine protocols
    def journal_all(self, timeout: float = 120.0) -> tuple[int, list[JournalItem]]:
        """``journal all``: IRJOURNAL_BEGIN / ITEM... / END. Returns (declared_total, items)."""
        self.flush_async()
        self.transport.send_command("journal all")
        begin = self.next_record(("IRJOURNAL_BEGIN,", "ERR:"), timeout, show_unmatched=False)
        if begin.startswith("ERR:"):
            self.wait_prompt(3.0)
            raise DeviceCommandError(begin)
        f = begin.split(",")
        if len(f) != 4:
            raise ProtocolError(f"invalid journal header: {begin}")
        try:
            declared = int(f[1])
        except ValueError as exc:
            raise ProtocolError(f"invalid journal header: {begin}") from exc
        items: list[JournalItem] = []
        while True:
            line = self.next_record(("IRJOURNAL_ITEM,", "IRJOURNAL_END,", "IRJOURNAL_ERROR,", "ERR:"),
                                    timeout, show_unmatched=False)
            if line.startswith("IRJOURNAL_ITEM,"):
                items.append(JournalItem.parse(line)); continue
            self.wait_prompt(5.0)
            if line.startswith("IRJOURNAL_END,"):
                return declared, items
            raise DeviceCommandError(f"journal export failed: {line}")

    def photos_list(self, timeout:float=30.0)->tuple[int,list[PhotoListItem],int]:
        self.flush_async()
        self.transport.send_command("photos list")
        active=False
        try:
            begin=self.next_record(("IRPHOTOS_BEGIN,","ERR:"),timeout,show_unmatched=False)
            if begin.startswith("ERR:"):
                self.wait_prompt(3.0)
                raise DeviceCommandError(begin)
            active=True
            f=begin.split(",")
            if len(f)!=3 or f[1]!="1": raise ProtocolError(f"unsupported photo list header: {begin}")
            expected=int(f[2]); items=[]
            while True:
                line=self.next_record(("IRPHOTOS_ITEM,","IRPHOTOS_END,","IRPHOTOS_ABORT,","ERR:"),timeout,show_unmatched=False)
                if line.startswith("ERR:"): raise DeviceCommandError(line)
                if line.startswith("IRPHOTOS_ITEM,"):
                    items.append(PhotoListItem.parse(line)); continue
                if line.startswith("IRPHOTOS_ABORT,"):
                    active=False
                    self.wait_prompt(3.0)
                    raise ProtocolError(f"photo list aborted: {line}")
                e=line.split(",")
                if len(e)!=3: raise ProtocolError(f"invalid list trailer: {line}")
                sent,skipped=int(e[1]),int(e[2])
                active=False
                self.wait_prompt(5.0)
                if sent!=len(items) or sent+skipped!=expected: raise ProtocolError("photo list item count mismatch")
                return expected,items,skipped
        except ConnectionLost:
            raise
        except Exception as exc:
            if active:
                recovered=False
                try:
                    self.transport.write(CANCEL)
                    deadline=time.monotonic()+8.0
                    while time.monotonic()<deadline:
                        line=self.transport.read_line(max(0.1,deadline-time.monotonic()))
                        if line.startswith(("IRPHOTOS_ABORT,","IRPHOTOS_END,")):
                            self.wait_prompt(3.0)
                            recovered=True
                            break
                except Exception:
                    recovered=False
                if not recovered and not isinstance(exc,TransportError):
                    raise TransportError("photo-list protocol failed and CLI recovery was not confirmed") from exc
            raise
