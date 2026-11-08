"""Unified exceptions and process exit codes for Incident Recorder Host Tool."""

class HostToolError(Exception):
    exit_code = 1
    #: True when the text of this error was already printed for the user
    #: (for example the device's own "ERR: ..." line), so callers must not
    #: print it a second time.
    shown = False

class ConfigurationError(HostToolError):
    pass

class TransportError(HostToolError):
    exit_code = 4

class ConnectionLost(TransportError):
    def __init__(self, message: str, partial: bytes = b""):
        super().__init__(message)
        self.partial = partial

class ReadTimeout(TransportError):
    def __init__(self, message: str, partial: bytes = b""):
        super().__init__(message)
        self.partial = partial

class ProtocolError(HostToolError):
    pass

class DeviceCommandError(HostToolError):
    def __init__(self, message: str = "", *, shown: bool = False):
        super().__init__(message)
        self.shown = shown

class PartialExecutionError(HostToolError):
    exit_code = 2

class NoPhotosError(HostToolError):
    exit_code = 3
