"""Sanity checks for runtime dependencies and YAML integrity."""
from __future__ import annotations

import importlib
import os
import sys


REQUIRED = [
    ('numpy',   True),
    ('scipy',   True),
    ('yaml',    True),       # pyyaml
    ('cv2',     True),       # opencv-python
    ('flask',   False),      # optional (UI only)
    ('open3d',  False),      # optional, fallback reader exists
    ('matplotlib', False),   # optional, visualization
]


def check_deps():
    print("== Dependency check ==")
    all_ok = True
    for mod, required in REQUIRED:
        try:
            m = importlib.import_module(mod)
            ver = getattr(m, '__version__', '?')
            print(f"  OK {mod:14s} {ver}")
        except ImportError as e:
            status = 'MISSING (required)' if required else 'MISSING (optional)'
            print(f"  {status} {mod:14s}  — {e}")
            if required:
                all_ok = False
    return all_ok


def check_configs():
    print("\n== Config files ==")
    from ..common.board_definition import BoardDef
    paths = [
        '/root/sam_ws/config/board_a_r2_params.yaml',
        '/root/sam_ws/config/board_b_r3_params.yaml',
        '/root/sam_ws/config/stage1_config.yaml',
        '/root/sam_ws/config/stage2_config.yaml',
        '/root/sam_ws/config/unit_mapping.yaml',
    ]
    all_ok = True
    for p in paths:
        if not os.path.exists(p):
            print(f"  MISSING: {p}")
            all_ok = False
            continue
        print(f"  OK: {p}")
    # Parse boards
    for p in [paths[0], paths[1]]:
        try:
            bd = BoardDef.load(p)
            print(f"  Parsed {bd.name}: charuco {bd.charuco.squares_x}×{bd.charuco.squares_y}, "
                  f"{len(bd.cones)} cones")
        except Exception as e:
            print(f"  FAIL parse {p}: {e}")
            all_ok = False
    return all_ok


def main():
    ok_deps = check_deps()
    ok_cfg  = check_configs()
    if ok_deps and ok_cfg:
        print("\nAll checks passed.")
        sys.exit(0)
    print("\nSome checks failed.", file=sys.stderr)
    sys.exit(1)


if __name__ == '__main__':
    main()
