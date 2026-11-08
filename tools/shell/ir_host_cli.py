#!/usr/bin/env python3
"""Compatibility launcher; the canonical entry point is `python -m ir_host`."""
import sys
from ir_host.shell import main

if __name__ == "__main__":
    argv = list(sys.argv[1:])
    # The previous launcher required a trailing `shell` subcommand. Interactive
    # mode is now the default, so accept and discard that legacy token.
    if argv and argv[-1].lower() == "shell":
        argv.pop()
    raise SystemExit(main(argv))
