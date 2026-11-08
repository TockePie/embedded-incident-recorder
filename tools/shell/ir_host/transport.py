"""Single-reader UART transport with buffering, deadlines and reconnect support."""
from __future__ import annotations
import threading, time
from .errors import ConnectionLost, ReadTimeout, TransportError

class SerialTransport:
    """Exactly one background thread reads the serial object; consumers share a condition buffer."""
    def __init__(self, serial_port, read_timeout: float = 1.0):
        self.serial = serial_port
        self.read_timeout = read_timeout
        self._cv = threading.Condition()
        self._buffer = bytearray()
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._failure: Exception | None = None
        self._closed = False
        self._start_reader()

    @staticmethod
    def list_ports() -> list[dict]:
        try:
            from serial.tools import list_ports
        except ImportError as exc:
            raise TransportError("pyserial is required: pip install pyserial") from exc
        result = []
        for p in list_ports.comports():
            result.append({"device": p.device, "description": p.description or "",
                           "manufacturer": p.manufacturer or "", "vid": p.vid, "pid": p.pid,
                           "hwid": p.hwid or ""})
        return result

    @classmethod
    def choose_port(cls, requested: str = "auto") -> str:
        ports = cls.list_ports()
        if requested != "auto":
            if any(p["device"] == requested for p in ports): return requested
            # Explicit path is allowed even when list_ports cannot enumerate it (e.g. stable symlink).
            return requested
        ftdi = [p for p in ports if p["vid"] == 0x0403 or "ft232" in p["description"].lower()
                or "ftdi" in p["manufacturer"].lower() or "ftdi" in p["description"].lower()]
        if len(ftdi) == 1: return ftdi[0]["device"]
        if len(ports) == 1: return ports[0]["device"]
        if not ports: raise TransportError("no serial ports found; connect the FT232 USB-UART adapter")
        names = ", ".join(p["device"] for p in ports)
        raise TransportError(f"cannot uniquely identify FT232; available ports: {names}; set port explicitly")

    @classmethod
    def open(cls, port: str, baud: int, read_timeout: float = 1.0) -> "SerialTransport":
        try: import serial
        except ImportError as exc: raise TransportError("pyserial is required: pip install pyserial") from exc
        actual = cls.choose_port(port)
        try:
            sp = serial.Serial(actual, baudrate=baud, timeout=read_timeout, write_timeout=3.0)
        except Exception as exc: raise ConnectionLost(f"cannot open {actual}: {exc}") from exc
        return cls(sp, read_timeout)

    @property
    def is_open(self) -> bool:
        return not self._closed and bool(getattr(self.serial, "is_open", True)) and self._failure is None

    def _start_reader(self) -> None:
        self._stop.clear()
        serial_obj=self.serial
        self._reader = threading.Thread(target=self._reader_loop,args=(serial_obj,),name="ir-uart-reader",daemon=True)
        self._reader.start()

    def _reader_loop(self,serial_obj) -> None:
        while not self._stop.is_set() and serial_obj is self.serial:
            try:
                available = getattr(serial_obj, "in_waiting", 0)
                # Read one byte when idle so the first byte of a 256-byte photo
                # chunk is delivered immediately; bulk-read bytes already queued.
                data = serial_obj.read(min(512, available) if available else 1)
                if data:
                    with self._cv:
                        if serial_obj is not self.serial: return
                        self._buffer.extend(data)
                        self._cv.notify_all()
            except Exception as exc:
                with self._cv:
                    if serial_obj is self.serial:
                        self._failure = exc
                        self._cv.notify_all()
                return

    def write(self, data: bytes) -> None:
        if self._closed: raise ConnectionLost("serial transport is closed")
        if self._failure: raise ConnectionLost(f"UART read failed: {self._failure}")
        try:
            count = self.serial.write(data)
            if count != len(data): raise TransportError(f"short UART write: {count}/{len(data)}")
            self.serial.flush()
        except Exception as exc:
            with self._cv: self._failure = exc; self._cv.notify_all()
            raise ConnectionLost(f"UART write failed: {exc}") from exc

    def send_command(self, command: str) -> None:
        if not command or "\r" in command or "\n" in command:
            raise ValueError("one non-empty command line is required")
        self.write(command.encode("ascii") + b"\r\n")

    def _take(self, count: int, deadline: float, idle_timeout: float | None = None) -> bytes:
        out = bytearray(); last_data = time.monotonic()
        with self._cv:
            while len(out) < count:
                if self._buffer:
                    n = min(count - len(out), len(self._buffer))
                    out.extend(self._buffer[:n]); del self._buffer[:n]
                    last_data = time.monotonic()
                    continue
                if self._failure:
                    raise ConnectionLost(f"UART disconnected: {self._failure}", bytes(out))
                if self._closed:
                    raise ConnectionLost("UART transport is closed",bytes(out))
                now = time.monotonic()
                if now >= deadline or (idle_timeout is not None and now-last_data >= idle_timeout):
                    raise ReadTimeout(f"received {len(out)} of {count} bytes before timeout", bytes(out))
                wait_for = deadline - now
                if idle_timeout is not None: wait_for = min(wait_for, idle_timeout-(now-last_data))
                self._cv.wait(max(0.001, wait_for))
        return bytes(out)

    def read_exact(self, count: int, timeout: float, idle_timeout: float | None = None) -> bytes:
        if count < 0: raise ValueError("count must be >= 0")
        if count == 0: return b""
        overall_timeout = timeout if idle_timeout is None else 24.0 * 60.0 * 60.0
        return self._take(count, time.monotonic()+overall_timeout, idle_timeout)

    def read_line_bytes(self, timeout: float = 5.0, max_bytes: int = 2048) -> bytes:
        deadline = time.monotonic()+timeout
        out = bytearray()
        while time.monotonic() < deadline:
            try: byte = self._take(1, deadline)
            except ReadTimeout as exc: raise ReadTimeout("timeout waiting for UART line",bytes(out)+exc.partial) from exc
            except ConnectionLost as exc: raise ConnectionLost(str(exc),bytes(out)+exc.partial) from exc
            out += byte
            if byte == b"\n": return bytes(out)
            if len(out) > max_bytes: raise TransportError(f"UART line exceeds {max_bytes} bytes")
        raise ReadTimeout("timeout waiting for UART line",bytes(out))

    def read_line(self, timeout: float = 5.0, max_bytes: int = 2048) -> str:
        return self.read_line_bytes(timeout, max_bytes).decode("utf-8", errors="replace").rstrip("\r\n\x00")

    def drain(self) -> bytes:
        """Atomically remove and return everything that is currently buffered.

        Used before every command so that stale prompts and unsolicited device
        notices can never be mistaken for the response to the next command."""
        with self._cv:
            data = bytes(self._buffer)
            self._buffer.clear()
        return data

    def unread(self, data: bytes) -> None:
        """Put bytes back at the front of the receive buffer."""
        if not data:
            return
        with self._cv:
            self._buffer[0:0] = data
            self._cv.notify_all()

    def read_line_or_marker(self, markers: tuple[bytes, ...], timeout: float,
                            max_bytes: int = 2048) -> tuple[bytes, bytes | None]:
        """Read until LF (returns marker=None) or until the data ends with one of
        ``markers`` (e.g. the unterminated CLI prompt ``IR> ``)."""
        deadline = time.monotonic() + timeout
        out = bytearray()
        while time.monotonic() < deadline:
            try: out += self._take(1, deadline)
            except ReadTimeout as exc: raise ReadTimeout("timeout waiting for UART line", bytes(out) + exc.partial) from exc
            except ConnectionLost as exc: raise ConnectionLost(str(exc), bytes(out) + exc.partial) from exc
            if out.endswith(b"\n"):
                return bytes(out), None
            for marker in markers:
                if out.endswith(marker):
                    return bytes(out), marker
            if len(out) > max_bytes: raise TransportError(f"UART line exceeds {max_bytes} bytes")
        raise ReadTimeout("timeout waiting for UART line", bytes(out))

    def read_until(self, marker: bytes, timeout: float, max_bytes: int = 1024*1024) -> bytes:
        deadline = time.monotonic()+timeout; out=bytearray()
        while time.monotonic() < deadline:
            try: out += self._take(1, deadline)
            except ReadTimeout as exc: raise ReadTimeout(f"timeout waiting for {marker!r}",bytes(out)+exc.partial) from exc
            except ConnectionLost as exc: raise ConnectionLost(str(exc),bytes(out)+exc.partial) from exc
            if out.endswith(marker): return bytes(out)
            if len(out) > max_bytes: raise TransportError("UART response exceeded safety limit")
        raise ReadTimeout(f"timeout waiting for {marker!r}",bytes(out))

    def read_until_any(self, markers: tuple[bytes, ...], timeout: float, max_bytes: int = 1024*1024) -> tuple[bytes, bytes]:
        deadline = time.monotonic() + timeout
        out = bytearray()
        while time.monotonic() < deadline:
            try: out += self._take(1, deadline)
            except ReadTimeout as exc: raise ReadTimeout(f"timeout waiting for one of {markers!r}",bytes(out)+exc.partial) from exc
            except ConnectionLost as exc: raise ConnectionLost(str(exc),bytes(out)+exc.partial) from exc
            for marker in markers:
                if out.endswith(marker):
                    return bytes(out), marker
            if len(out) > max_bytes:
                raise TransportError("UART response exceeded safety limit")
        raise ReadTimeout(f"timeout waiting for one of {markers!r}",bytes(out))

    def close(self) -> None:
        if self._closed: return
        self._closed = True; self._stop.set()
        try: self.serial.close()
        except Exception: pass
        if self._reader and self._reader is not threading.current_thread(): self._reader.join(timeout=self.read_timeout+1.0)
        with self._cv: self._cv.notify_all()

    def reconnect(self, port: str, baud: int, timeout: float, interval: float = 1.0) -> None:
        self.close()
        deadline = time.monotonic() + timeout
        last_error = "not attempted"
        while time.monotonic() <= deadline:
            try:
                import serial
                actual = self.choose_port(port)
                replacement = serial.Serial(actual, baudrate=baud, timeout=self.read_timeout, write_timeout=3.0)
                self.serial = replacement
                self._closed = False
                self._failure = None
                self._buffer.clear()
                self._reader = None
                self._start_reader()
                return
            except Exception as exc:
                last_error = str(exc)
                if time.monotonic() >= deadline: break
                time.sleep(min(interval,max(0.0,deadline-time.monotonic())))
        raise ConnectionLost(f"reconnect failed after {timeout:.1f}s: {last_error}")
