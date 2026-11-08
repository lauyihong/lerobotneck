"""Load the robot description configuration (hardware/wallerobot_v0_1/robot.yaml), optionally deep-merged with an override.

    import robot_config
    cfg = robot_config.load()                 # default file (+ $WALLEROBOT_CONFIG if set)
    cfg = robot_config.load("my.yaml")        # default + my.yaml
    robot_config.path(cfg["calibration"]["neck"])   # -> absolute Path (paths in the config are relative to the repo root)

The override file only needs the keys it changes; mappings are merged recursively, everything else (lists, scalars)
is replaced. `load()` with the same arguments returns the same (cached) dict, so the builders, which import each other,
all see one configuration. `use(override)` sets the override for the whole process (build_all.py --config does this).
"""
import copy
import os
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / "hardware/wallerobot_v0_1/robot.yaml"
_override = os.environ.get("WALLEROBOT_CONFIG") or None
_cache = {}


def _merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


def use(override):
    """Set the process-wide override file (None = defaults only)."""
    global _override
    _override = str(override) if override else None
    _cache.clear()


def load(override=None):
    over = str(override) if override else _override
    if over not in _cache:
        cfg = yaml.safe_load(DEFAULT.read_text(encoding="utf-8"))
        if over:
            _merge(cfg, yaml.safe_load(Path(over).read_text(encoding="utf-8")) or {})
        _cache[over] = cfg
    return copy.deepcopy(_cache[over])


def path(p) -> Path:
    """Config path (relative to the repo root, or absolute) -> absolute Path."""
    p = Path(p)
    return p if p.is_absolute() else ROOT / p


def enabled_arms(cfg):
    return [s for s in ("left", "right") if cfg["arms"][s].get("enabled", True)]
