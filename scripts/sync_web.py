#!/usr/bin/env python3
"""Copies web/ -> data/web/ (delete-then-copy).

data/ is generated from web/ and is never hand-edited (CLAUDE.md) - this
script is the only thing that should write to it. Run before flashing the
filesystem image:

    python scripts/sync_web.py
    pio run -e esp32s3 -t uploadfs
"""

import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "web"
DST = ROOT / "data" / "web"


def main() -> int:
    if not SRC.is_dir():
        print(f"error: {SRC} does not exist", file=sys.stderr)
        return 1

    if DST.exists():
        shutil.rmtree(DST)
    shutil.copytree(SRC, DST)

    files = sorted(p.relative_to(ROOT) for p in DST.rglob("*") if p.is_file())
    print(f"Synced {len(files)} file(s) from web/ to data/web/:")
    for f in files:
        print(f"  {f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
