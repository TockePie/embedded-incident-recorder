"""Strict JSON configuration for the PC-side Host Tool."""
from __future__ import annotations
import json, os
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any
from .errors import ConfigurationError

@dataclass
class HostConfig:
    port: str = "auto"
    baud: int = 115200
    read_timeout: float = 1.0
    command_timeout: float = 5.0
    photo_timeout: float = 5.0  # idle timeout: maximum silence between received bytes
    retries: int = 3            # retries after the first attempt
    rgb565_byte_order: str = "big"
    keep_rgb565: bool = False
    keep_metadata: bool = False  # write event_N_WxH.json next to each PNG
    output_dir: str = "incident_photos"
    png_scale: int = 2
    overwrite: bool = False
    auto_reconnect: bool = True
    reconnect_timeout: float = 30.0
    reconnect_interval: float = 1.0
    log_file: str | None = None

    @classmethod
    def default_path(cls) -> Path:
        return Path.home() / ".incident_recorder" / "config.json"

    @classmethod
    def load(cls, path: str | Path | None = None) -> "HostConfig":
        p = Path(path) if path else cls.default_path()
        if not p.exists():
            return cls()
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ConfigurationError(f"cannot load settings {p}: {exc}") from exc
        if not isinstance(data, dict):
            raise ConfigurationError("settings root must be a JSON object")
        allowed = {f.name for f in fields(cls)}
        unknown = set(data) - allowed
        if unknown:
            raise ConfigurationError("unknown settings: " + ", ".join(sorted(unknown)))
        try:
            cfg = cls(**data)
        except TypeError as exc:
            raise ConfigurationError(str(exc)) from exc
        cfg.validate()
        return cfg

    def validate(self) -> None:
        if not isinstance(self.port, str) or not self.port.strip():
            raise ConfigurationError("port must be 'auto' or a non-empty port name")
        if self.port != "auto" and any(c.isspace() for c in self.port):
            raise ConfigurationError("port must not contain whitespace")
        if type(self.baud) is not int or not 1200 <= self.baud <= 2_000_000:
            raise ConfigurationError("baud must be an integer in range 1200..2000000")
        for name, value, low, high in (
            ("read_timeout", self.read_timeout, 0.05, 30.0),
            ("command_timeout", self.command_timeout, 0.2, 300.0),
            ("photo_timeout", self.photo_timeout, 0.2, 120.0),
            ("reconnect_timeout", self.reconnect_timeout, 0.0, 600.0),
            ("reconnect_interval", self.reconnect_interval, 0.1, 30.0),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high:
                raise ConfigurationError(f"{name} must be in range {low}..{high}")
        if type(self.retries) is not int or not 0 <= self.retries <= 3:
            raise ConfigurationError("retries must be an integer in range 0..3 (firmware IRPHOTO4 limit)")
        if self.rgb565_byte_order not in ("big", "little"):
            raise ConfigurationError("rgb565_byte_order must be 'big' or 'little'")
        if type(self.keep_rgb565) is not bool or type(self.keep_metadata) is not bool or type(self.overwrite) is not bool or type(self.auto_reconnect) is not bool:
            raise ConfigurationError("keep_rgb565, keep_metadata, overwrite and auto_reconnect must be booleans")
        if not isinstance(self.output_dir, str) or not self.output_dir.strip():
            raise ConfigurationError("output_dir must be a non-empty path")
        if type(self.png_scale) is not int or not 1 <= self.png_scale <= 16:
            raise ConfigurationError("png_scale must be an integer in range 1..16")
        if self.log_file is not None and (not isinstance(self.log_file, str) or not self.log_file.strip()):
            raise ConfigurationError("log_file must be a non-empty path string or null")

    def save(self, path: str | Path | None = None) -> Path:
        self.validate()
        target = Path(path) if path else self.default_path()
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_suffix(target.suffix + ".tmp")
        try:
            with temp.open("w", encoding="utf-8", newline="\n") as f:
                json.dump(asdict(self), f, indent=2)
                f.write("\n"); f.flush(); os.fsync(f.fileno())
            os.replace(temp, target)
            return target
        except Exception:
            temp.unlink(missing_ok=True)
            raise

    def set_value(self, key: str, raw: str) -> None:
        if key not in {f.name for f in fields(self)}:
            raise ConfigurationError(f"unknown setting '{key}'")
        current = getattr(self, key)
        try:
            if isinstance(current, bool):
                value = raw.lower()
                if value not in ("true", "false", "1", "0", "yes", "no", "on", "off"):
                    raise ValueError("expected true/false")
                parsed: Any = value in ("true", "1", "yes", "on")
            elif isinstance(current, int): parsed = int(raw, 10)
            elif isinstance(current, float): parsed = float(raw)
            elif key == "log_file": parsed = None if raw.lower() in ("null", "none") else raw
            else: parsed = raw
            old = current; setattr(self, key, parsed)
            try: self.validate()
            except Exception:
                setattr(self, key, old); raise
        except (ValueError, TypeError) as exc:
            raise ConfigurationError(f"invalid value for {key}: {exc}") from exc
