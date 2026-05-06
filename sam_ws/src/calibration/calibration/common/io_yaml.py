"""YAML I/O with optional PyYAML dependency and graceful fallback."""
from __future__ import annotations

import os
from typing import Any

try:
    import yaml  # type: ignore
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False


def load_yaml(path: str) -> dict:
    if not _HAS_YAML:
        raise ImportError("PyYAML is required. Install with: pip install pyyaml")
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def save_yaml(path: str, data: Any) -> None:
    if not _HAS_YAML:
        raise ImportError("PyYAML is required. Install with: pip install pyyaml")
    os.makedirs(os.path.dirname(path), exist_ok=True) if os.path.dirname(path) else None
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True, indent=2)


def sha256_file(path: str) -> str:
    import hashlib
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 16), b''):
            h.update(chunk)
    return h.hexdigest()
