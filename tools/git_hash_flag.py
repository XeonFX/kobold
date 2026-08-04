#!/usr/bin/env python3
"""Emit a -D FW_GIT_HASH=0x... build flag for PlatformIO.

Invoked from platformio.ini via the `!` shell-command syntax. The hash is
reported in the `version` message so the bridge can log exactly which build is
running on each board — and so a firmware flashed from an uncommitted working
tree is identifiable as such.
"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main() -> None:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short=8", "HEAD"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        value = int(out.stdout.strip(), 16)

        dirty = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        if dirty:
            # Flip the top bit so a build from a dirty tree is obvious in the
            # logs rather than masquerading as a clean commit.
            value |= 0x80000000
    except (subprocess.CalledProcessError, FileNotFoundError, ValueError):
        value = 0

    sys.stdout.write(f"-D FW_GIT_HASH=0x{value:08X}")


if __name__ == "__main__":
    main()
