#!/usr/bin/env python3
"""Summarize GCC -fstack-usage (.su) files produced by STM32CubeIDE."""

import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", help=".su files or directories to scan")
    parser.add_argument("-n", "--top", type=int, default=25,
                        help="number of largest function frames to show")
    args = parser.parse_args()

    files = []
    for item in args.paths:
        path = Path(item)
        if path.is_dir():
            files.extend(path.rglob("*.su"))
        elif path.suffix == ".su" and path.is_file():
            files.append(path)

    rows = []
    for file in files:
        for line_number, line in enumerate(file.read_text(errors="replace").splitlines(), 1):
            fields = line.split("\t")
            if len(fields) < 3:
                continue
            try:
                size = int(fields[1].strip())
            except ValueError:
                continue
            rows.append((size, fields[2].strip(), fields[0].strip(), file, line_number))

    if not rows:
        raise SystemExit("No valid .su records found. Enable -fstack-usage and rebuild.")

    rows.sort(reverse=True, key=lambda row: row[0])
    print("frame_bytes\tclassification\tfunction\tsource_file")
    for size, classification, function, file, _ in rows[:max(args.top, 1)]:
        print(f"{size}\t{classification}\t{function}\t{file}")

    dynamic = sum(1 for row in rows if row[1].startswith("dynamic"))
    print(f"\nfunctions={len(rows)} dynamic_or_dynamic_bounded={dynamic}")
    print("Note: largest individual frames are not the call-chain maximum; inspect call paths and ISR nesting.")


if __name__ == "__main__":
    main()
