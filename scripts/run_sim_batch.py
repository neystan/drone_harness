#!/usr/bin/env python3
"""从源码运行批测，确保使用当前 drone_harness。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drone_harness.testing.batch import main

if __name__ == "__main__":
    raise SystemExit(main())
