"""Interactive, one-shot and script command shell for Incident Recorder."""
from __future__ import annotations
import getpass, json, os, re, shlex, sys, time
from pathlib import Path
from .config import HostConfig
from .errors import (ConnectionLost, DeviceCommandError, HostToolError, NoPhotosError,
                     ReadTimeout, TransportError)
from .help_texts import TOPICS
from .photo import PhotoEngine
from .protocol import (CONFIRM_PROMPT, PROMPT, ProtocolParser, check_command_text, strip_ansi)
from .transport import SerialTransport

SAVE_WAIT_S = 15.0          # how long to wait for "persistent save complete"
ERASE_IDLE_S = 300.0        # max silence while the device erases flash
PASSWORD_HINT = "password must be 8..32 printable characters without spaces"


def read_secret(prompt: str) -> str:
    """Read a password showing '*' for every typed character.

    Falls back to getpass (no echo at all) when stdin is not a real terminal."""
    if not sys.stdin.isatty():
        return getpass.getpass(prompt)
    out = sys.stdout
    chars: list[str] = []
    out.write(prompt); out.flush()
    if os.name == "nt":
        import msvcrt
        while True:
            ch = msvcrt.getwch()
            if ch in ("\r", "\n"):
                out.write("\n"); out.flush(); return "".join(chars)
            if ch == "\x03": raise KeyboardInterrupt
            if ch in ("\x00", "\xe0"): msvcrt.getwch(); continue      # arrow/function key
            if ch == "\b":
                if chars: chars.pop(); out.write("\b \b"); out.flush()
                continue
            if ch.isprintable():
                chars.append(ch); out.write("*"); out.flush()
    import termios, tty
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)                      # no echo, Ctrl+C still raises KeyboardInterrupt
        while True:
            ch = sys.stdin.read(1)
            if ch in ("\r", "\n"):
                out.write("\n"); out.flush(); return "".join(chars)
            if ch == "":
                raise EOFError
            if ch in ("\x7f", "\b"):
                if chars: chars.pop(); out.write("\b \b"); out.flush()
                continue
            if ch == "\x04" and not chars: raise EOFError
            if ch.isprintable():
                chars.append(ch); out.write("*"); out.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def tokenize(line: str) -> list[str]:
    """Split a command line. Quotes group words; backslashes (Windows paths) and '#'
    (allowed inside passwords) are kept literally."""
    try:
        tokens = shlex.split(line, comments=False, posix=False)
    except ValueError as exc:
        raise ValueError(f"command syntax: {exc}") from exc
    return [t[1:-1] if len(t) >= 2 and t[0] == t[-1] and t[0] in "\"'" else t for t in tokens]


class HostShell:
    def __init__(self, config:HostConfig, config_path:Path|None=None, output=print):
        self.config=config; self.config_path=config_path; self.output=output
        self.transport=None; self.parser=None; self.photo=None; self._log_stream=None
        self._apply_log_setting()

    # ----------------------------------------------------------------- logging
    def _apply_log_setting(self):
        """Make the session log follow config.log_file (None = no logging)."""
        if self._log_stream:
            self._log_stream.close(); self._log_stream=None
        if self.config.log_file:
            target=Path(self.config.log_file); target.parent.mkdir(parents=True,exist_ok=True)
            self._log_stream=target.open("a",encoding="utf-8")

    # ------------------------------------------------------------------ output
    def _emit(self,text:str=""):
        self.output(text)
        if self._log_stream:
            self._log_stream.write(text+"\n"); self._log_stream.flush()

    def _report(self, exc: BaseException) -> None:
        """Print an error once (not again if the device text was already shown)."""
        if not getattr(exc, "shown", False):
            self._emit(f"[error] {exc}")

    @staticmethod
    def _ask_secret(prompt: str) -> str:
        return read_secret(prompt)

    @staticmethod
    def _validate_password(password: str) -> None:
        if not 8 <= len(password) <= 32 or any(not 0x21 <= ord(c) <= 0x7E for c in password):
            raise ValueError(PASSWORD_HINT)

    # -------------------------------------------------------------- connection
    def _connect(self, port:str|None=None):
        if self.transport and self.transport.is_open:
            self._emit("[host] already connected")
            return
        if self.transport:
            self.transport.close()
            self.transport=None
        requested=port or self.config.port
        if port: self.config.port=port
        transport=SerialTransport.open(requested,self.config.baud,self.config.read_timeout)
        try:
            parser=ProtocolParser(transport,self.config.command_timeout,on_text=self._emit)
            if not parser.sync():
                raise ConnectionLost("CLI prompt not observed or synchronization response was ambiguous")
        except Exception:
            transport.close()
            raise ConnectionLost(f"opened {requested}, but the STM32 CLI prompt could not be synchronized")
        except BaseException:
            transport.close()
            raise
        photo=PhotoEngine(parser,self.config,log=self._emit)
        self.transport=transport
        self.parser=parser
        self.photo=photo
        self._emit(f"[host] connected: {self.transport.serial.port if hasattr(self.transport.serial,'port') else requested} @ {self.config.baud}")

    def _disconnect(self):
        was_connected = self.transport is not None
        if self.transport: self.transport.close()
        self.transport=self.parser=self.photo=None
        if was_connected: self._emit("[host] disconnected")

    def _ensure(self):
        if self.transport is None:
            self._connect()
        elif not self.transport.is_open:
            if not self._reconnect_and_sync():
                raise ConnectionLost("UART is disconnected and automatic reconnect failed")

    def _reconnect_and_sync(self)->bool:
        if not self.config.auto_reconnect or not self.transport: return False
        try:
            self._emit("[host] connection lost; reconnecting...")
            self.transport.reconnect(self.config.port,self.config.baud,self.config.reconnect_timeout,self.config.reconnect_interval)
            return self.parser.sync()
        except Exception: return False

    def _recover_after_interrupt(self):
        """Ctrl+C: stop whatever the device is doing and get back to a clean prompt."""
        self._emit("[host] interrupted")
        if self.transport is not None and self.parser is not None and self.transport.is_open:
            try:
                if not self.parser.sync():
                    self._emit("[host] the device did not answer; try 'reconnect'")
            except Exception as exc:
                self._emit(f"[host] could not resynchronise: {exc}; try 'reconnect'")

    @staticmethod
    def _is_safe_to_retry(command:str)->bool:
        c=command.strip().lower().split()
        if c==["status"] or c==["config"] or c==["journal"] or c==["journal","all"]: return True
        if len(c)==2 and c[0]=="journal" and c[1].isdigit(): return True
        if len(c)==3 and c[:2]==["journal","id"] and c[2].isdigit(): return True
        if c==["photos","list"]: return True
        return False

    # ----------------------------------------------------------- device access
    @staticmethod
    def _check_result(command:str, text:str):
        failed = bool(re.search(r"(?m)^ERR:", text))
        if command.strip().lower()=="photo selftest" and re.search(r"\bFAIL\b", text): failed=True
        if failed: raise DeviceCommandError(text, shown=True)

    def _await_save(self, text:str):
        """seal/unseal/config answer 'save started' at once; the real result arrives
        later as an unsolicited notice. Wait for it so it cannot get mixed into the
        answer of the next command."""
        if "save started" not in text.lower(): return
        notice=self.parser.wait_async(("persistent save complete","save failed","error"),SAVE_WAIT_S)
        if notice is None:
            self._emit("[host] the device is still saving; its confirmation will be shown before the next command")
            return
        self._emit(notice)
        if "error" in notice.lower() or "failed" in notice.lower():
            raise DeviceCommandError(notice, shown=True)

    def _device(self,command:str,timeout:float|None=None,secret:bool=False):
        check_command_text(command)
        self._ensure()
        try:
            result=self.parser.command(command,timeout=timeout,allow_device_error=True)
        except TransportError:
            if secret or not self._is_safe_to_retry(command) or not self._reconnect_and_sync(): raise
            result=self.parser.command(command,timeout=timeout,allow_device_error=True)
        if result: self._emit(result)
        self._check_result(command,result)
        self._await_save(result)
        return result

    def _password_command(self,verb:str,args:list[str]):
        if len(args)>1: raise ValueError(f"usage: {verb} [PASSWORD]")
        if args:
            password=args[0]
        else:
            password=self._ask_secret(f"{verb} password: ")
            if verb=="seal":
                if self._ask_secret("repeat password: ")!=password:
                    raise ValueError("passwords do not match; nothing was sent")
        self._validate_password(password)
        self._device(f"{verb} {password}",secret=True)

    def _stream_until_prompt(self, echo:str, timeout:float)->str:
        """Print device lines as they arrive (erase progress) until the prompt returns."""
        deadline=time.monotonic()+timeout
        want=echo.strip(); echoed=False; collected=[]
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0: raise ReadTimeout("timeout waiting for the device")
            raw,marker=self.transport.read_line_or_marker((PROMPT,),min(ERASE_IDLE_S,remaining))
            line=strip_ansi(raw).replace("\r","").replace("IR> ","").rstrip()
            if line.strip():
                if not echoed and line.strip()==want: echoed=True
                else: self._emit(line); collected.append(line)
            if marker is not None: return "\n".join(collected)

    def _erase(self,args:list[str]):
        if len(args)!=1 or args[0] not in ("config","media","all"):
            raise ValueError("usage: erase config|media|all")
        scope=args[0]
        password=self._ask_secret("Device password: ")
        self._validate_password(password)
        self._ensure()
        self._emit(f"> erase {scope} ********")
        text,marker=self.parser.exchange(f"erase {scope} {password}",timeout=self.config.command_timeout,
                                         extra_markers=(CONFIRM_PROMPT,))
        if text: self._emit(text)
        if marker==PROMPT:                       # the device refused (not sealed, wrong password ...)
            self._check_result("erase",text)
            return
        answer=input("Type YES to confirm the erase, anything else cancels: ").strip()
        late=self.parser.flush_async(show=False)
        if "cancelled" in late.lower():          # 30 s confirmation window expired meanwhile
            self._emit(late); return
        reply="YES" if answer=="YES" else "NO"
        self.transport.send_command(reply)
        result=self._stream_until_prompt(reply,1800.0)
        if re.search(r"(?m)\bERROR\b|^ERR:",result): raise DeviceCommandError(result,shown=True)

    # ------------------------------------------------------------- data export
    def _photos_list(self):
        self._ensure()
        try: expected,items,skipped=self.parser.photos_list()
        except TransportError:
            if not self._reconnect_and_sync(): raise
            expected,items,skipped=self.parser.photos_list()
        index_path=self.photo.save_index(expected,items,skipped)
        if not items:
            if expected==0 or skipped>=expected: raise NoPhotosError("device has no downloadable photos")
            raise NoPhotosError("no valid photos in device list")
        self._emit(f"[photos] available={len(items)} skipped={skipped} index_entries={expected}; index={index_path}")
        for x in items:
            self._emit(f"  id={x.event_id} {x.width}x{x.height} bytes={x.size} CRC32={x.crc32:08X}")
        return 0

    def _journal_all(self):
        self._ensure()
        try: declared,items=self.parser.journal_all()
        except TransportError:
            if not self._reconnect_and_sync(): raise
            declared,items=self.parser.journal_all()
        self._emit(f"[JOURNAL] records_exported={len(items)}")
        for r in items:
            self._emit(f"[JOURNAL RECORD] id={r.event_id} type={r.type} mode={r.mode} "
                       f"tick_ms={r.tick_ms} value={r.value} threshold={r.threshold}")
        if declared!=len(items):
            self._emit(f"[JOURNAL] warning: device announced {declared} records but sent {len(items)}")
        return 0

    def _photo_one(self,event_id:int):
        self._ensure()
        result=self.photo.download_event(event_id)
        self._emit(f"[photo] id={event_id} complete; chunks={result['chunks']} CRC32={result['crc32']} PNG={result['png']}")
        return 0

    # ----------------------------------------------------------------- routing
    @staticmethod
    def _positive_int(text:str, what:str, high:int|None=None)->int:
        if not text.isdigit() or int(text)<=0 or (high is not None and int(text)>high):
            raise ValueError(f"{what} must be a whole number" + (f" 1..{high}" if high else " greater than 0"))
        return int(text)

    def _device_route(self,tokens:list[str]):
        if not tokens: return 0
        root=tokens[0].lower(); args=tokens[1:]
        if root=="status":
            if args: raise ValueError("usage: status")
            self._device("status"); return 0
        if root=="config":
            if not args or args==["show"]: command="config"
            else: command="config "+" ".join(args)
            self._device(command); return 0
        if root=="journal":
            if not args: command="journal"
            elif args[0]=="recent":
                if len(args)>2: raise ValueError("usage: journal recent [N]")
                n=self._positive_int(args[1],"journal count",30) if len(args)==2 else 5
                command=f"journal {n}"
            elif args[0]=="id" and len(args)==2:
                command=f"journal id {self._positive_int(args[1],'journal event ID')}"
            elif args==["all"]: return self._journal_all()
            elif len(args)==1 and args[0].isdigit():
                command=f"journal {self._positive_int(args[0],'journal count',30)}"
            else: raise ValueError("usage: journal [N] | journal id ID | journal all")
            self._device(command); return 0
        if root=="photos":
            if not args: self._device("photos"); return 0
            if args==["list"]: return self._photos_list()
            if len(args)==1 and args[0].isdigit():
                self._device(f"photos {self._positive_int(args[0],'photo count',30)}"); return 0
            raise ValueError("usage: photos [N] | photos list")
        if root=="photo":
            if args==["all"]:
                self._ensure(); return self.photo.download_all()
            if args==["list"]: return self._photos_list()
            if len(args)==2 and args[0] in ("id","download","get"):
                return self._photo_one(self._positive_int(args[1],"event ID"))
            if args in (["status"],["selftest"]): self._device("photo "+args[0]); return 0
            raise ValueError("usage: photo id ID | photo all | photo list | photo status | photo selftest")
        if root in ("seal","unseal"): self._password_command(root,args); return 0
        if root=="erase": self._erase(args); return 0
        if root=="led" and args==["test"]: self._device("led test"); return 0
        self._device(" ".join(tokens)); return 0

    def _set_setting(self, args:list[str]):
        if len(args)!=2: raise ValueError("usage: settings set KEY VALUE")
        self.config.set_value(args[0],args[1])
        if args[0]=="log_file":
            self._apply_log_setting()
        if args[0] in ("port","baud","read_timeout"):
            self._disconnect()
        elif self.parser:
            self.photo=PhotoEngine(self.parser,self.config,log=self._emit)
        self._emit(f"[settings] {args[0]}={getattr(self.config,args[0])!r} (not saved; use 'settings save')")

    def _settings(self, sub:str, rest:list[str]):
        if sub=="show":
            if rest: raise ValueError("usage: settings [show]")
            for key,value in self.config.__dict__.items(): self._emit(f"{key}={value!r}")
        elif sub=="set": self._set_setting(rest)
        elif sub=="save":
            p=self.config.save(self.config_path); self._emit(f"[settings] saved: {p}")
        elif sub=="load":
            self._disconnect(); self.config=HostConfig.load(self.config_path); self._apply_log_setting(); self._emit("[settings] loaded")
        elif sub in ("reset","reset-settings"):
            self._disconnect(); self.config=HostConfig(); self._apply_log_setting(); self._emit("[settings] reset to defaults in memory; use 'settings save' to keep")
        else:
            raise ValueError("usage: settings [show | set KEY VALUE | save | load | reset]")

    def _host_route(self,tokens:list[str]):
        if not tokens: return 0
        cmd=tokens[0].lower(); args=tokens[1:]
        if cmd in ("exit","quit"): return "exit"
        if cmd in ("help","?"):
            topic=args[0].lower() if args else "host"
            text=TOPICS.get(topic)
            if text is None:
                raise ValueError("no help for '%s'; topics: %s"%(topic,", ".join(sorted(k for k in TOPICS if k not in ("host","overview","unseal","photos")))))
            self._emit(text.rstrip()); return 0
        if cmd=="ports":
            for p in SerialTransport.list_ports():
                ftdi=(p["vid"]==0x0403 or "ftdi" in p["description"].lower() or "ftdi" in p["manufacturer"].lower() or "ft232" in p["description"].lower())
                self._emit(f"{'[FT232] ' if ftdi else '         '}{p['device']} — {p['description']} (VID={p['vid']}, PID={p['pid']})")
            return 0
        if cmd=="connect":
            if len(args)>1: raise ValueError("usage: connect [PORT]")
            self._connect(args[0] if args else None); return 0
        if cmd=="disconnect":
            if args: raise ValueError("usage: disconnect")
            self._disconnect(); return 0
        if cmd=="reconnect":
            if args: raise ValueError("usage: reconnect")
            self._disconnect(); self._connect(); return 0
        if cmd=="settings":
            self._settings(args[0].lower() if args else "show", args[1:]); return 0
        # legacy spellings, still accepted
        if cmd=="set": self._set_setting(args); return 0
        if cmd in ("save","load"): self._settings(cmd,args); return 0
        if cmd=="reset-settings": self._settings("reset",args); return 0
        if cmd=="log":
            if not args or args==["show"]:
                self._emit(f"[log] {'on: '+str(self.config.log_file) if self._log_stream else 'off'}"); return 0
            if args[0]=="off":
                self.config.log_file=None; self._apply_log_setting()
                self._emit("[log] off (log_file=None; 'settings save' to keep this)"); return 0
            if args[0]=="on":
                if len(args)>2: raise ValueError("usage: log on [FILE]")
                self.config.log_file=args[1] if len(args)==2 else (self.config.log_file or "incident_recorder_session.log")
                self._apply_log_setting()
                self._emit(f"[log] on: {self.config.log_file} (log_file updated; 'settings save' to keep it for next start)"); return 0
            raise ValueError("usage: log on [FILE] | log off | log show")
        if cmd=="raw":
            if not args: raise ValueError("usage: raw COMMAND")
            command=" ".join(args)
            first=command.split()[0].lower()
            if first=="erase": raise ValueError("use the 'erase' command; it handles the password and the YES confirmation")
            self._device(command,secret=first in ("seal","unseal"))
            return 0
        # Non-host commands are routed to the STM32.
        return self._device_route(tokens)

    @staticmethod
    def _redact_command(tokens:list[str], original:str)->str:
        if tokens and tokens[0].lower() in ("seal","unseal") and len(tokens)>1:
            return tokens[0]+" ********"
        if tokens and tokens[0].lower()=="erase" and len(tokens)>2:
            return "erase "+tokens[1]+" [PASSWORD REDACTED]"
        if len(tokens)>1 and tokens[0].lower()=="raw" and tokens[1].lower() in ("seal","unseal","erase"):
            return "raw "+tokens[1]+" [ARGUMENTS REDACTED]"
        return original.strip()

    def execute_line(self,line:str):
        tokens=tokenize(line)
        if not tokens: return 0
        if self._log_stream:
            self._log_stream.write("irhost> "+self._redact_command(tokens,line)+"\n")
            self._log_stream.flush()
        return self._host_route(tokens)

    # ------------------------------------------------------------ entry points
    def _auto_connect(self):
        try: self._connect()
        except HostToolError as exc:
            self._report(exc)
            self._emit("[host] not connected. Use 'ports' to list serial ports and 'connect PORT' to open one.")

    def interactive(self)->int:
        try: import readline  # noqa: F401  (line editing/history where available)
        except ImportError: pass
        self._emit("Incident Recorder Host Tool — type 'help' for commands; 'exit' to quit.")
        self._auto_connect()
        while True:
            try:
                if self.parser is not None and self.transport is not None and self.transport.is_open:
                    self.parser.flush_async()          # show notices that arrived while idle
                line=input("irhost> ")
            except EOFError: break
            except KeyboardInterrupt:
                self.output(""); continue               # Ctrl+C at the prompt only clears the line
            try:
                result=self.execute_line(line)
                if result=="exit": break
            except KeyboardInterrupt:
                self._recover_after_interrupt()
            except Exception as exc: self._report(exc)
        self._disconnect()
        if self._log_stream: self._log_stream.close()
        return 0

    def run_command(self,command:str)->int:
        try:
            result=self.execute_line(command)
            return 0 if result in (None,"exit") else int(result)
        except HostToolError as exc:
            self._report(exc); return exc.exit_code
        except Exception as exc:
            self._report(exc); return 1

    def run_script(self,path:Path)->int:
        results=[]
        for number,line in enumerate(path.read_text(encoding="utf-8").splitlines(),1):
            stripped=line.strip()
            if not stripped or stripped.startswith("#"): continue
            code=self.run_command(stripped); results.append(code)
            if code==4: break
        if any(c==4 for c in results): return 4
        if any(c==2 for c in results): return 2
        return 1 if any(c not in (0,3) for c in results) else (3 if results and all(c==3 for c in results) else 0)

    def close(self):
        if self.transport: self.transport.close()
        self.transport=self.parser=self.photo=None
        if self._log_stream: self._log_stream.close(); self._log_stream=None

def main(argv=None)->int:
    import argparse
    parser=argparse.ArgumentParser(prog="python -m ir_host",description="Incident Recorder Host CLI")
    parser.add_argument("--config",type=Path,help="path to Host Tool JSON settings")
    parser.add_argument("--port",help="serial port or auto; overrides JSON for this run")
    parser.add_argument("--baud",type=int,help="UART baud rate; overrides JSON for this run")
    parser.add_argument("--read-timeout",type=float,help="serial reader timeout")
    parser.add_argument("--command-timeout",type=float,help="device command timeout")
    parser.add_argument("--photo-timeout",type=float,help="photo idle timeout between bytes")
    parser.add_argument("--retries",type=int,help="retries after initial photo attempt")
    parser.add_argument("--output-dir",help="photo output directory")
    parser.add_argument("--rgb565-byte-order",choices=("big","little"))
    parser.add_argument("--keep-rgb565",action="store_true",help="also keep verified raw RGB565 files")
    parser.add_argument("--keep-metadata",action="store_true",help="also write event_N_WxH.json next to each PNG")
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument("-c","--command",help="execute one Host/device command")
    modes.add_argument("--script",type=Path,help="execute commands from a UTF-8 text file")
    args=parser.parse_args(argv)
    try:
        cfg=HostConfig.load(args.config)
        for key in ("port","baud","read_timeout","command_timeout","photo_timeout","retries","output_dir","rgb565_byte_order"):
            value=getattr(args,key,None)
            if value is not None: setattr(cfg,key,value)
        if args.keep_rgb565: cfg.keep_rgb565=True
        if args.keep_metadata: cfg.keep_metadata=True
        cfg.validate()
        shell=HostShell(cfg,args.config)
        try:
            if args.command is not None: return shell.run_command(args.command)
            if args.script is not None:
                if not args.script.is_file(): raise FileNotFoundError(args.script)
                return shell.run_script(args.script)
            return shell.interactive()
        finally:
            shell.close()
    except HostToolError as exc:
        print(f"[error] {exc}",file=sys.stderr); return exc.exit_code
    except KeyboardInterrupt:
        print("\n[host] interrupted",file=sys.stderr); return 1
    except Exception as exc:
        print(f"[error] {exc}",file=sys.stderr); return 1
