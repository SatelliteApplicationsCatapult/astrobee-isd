"""Bag folder naming.

Layout produced per run:

    <save_dir>/<YYYYmmdd>_<HHMMSS>_<NNN>_<suffix>/
        <same name>.bag
        SUCCESS | FAILURE
        metadata.json
"""

import os
import re
from datetime import datetime
from typing import Optional

_UNSAFE = re.compile(r'[^A-Za-z0-9_\-]')
_FOLDER = re.compile(r'^\d{8}_\d{6}_(\d{3})')


def sanitise_suffix(raw: str) -> str:
    """Spaces to underscores, drop anything else awkward in a path."""
    cleaned = _UNSAFE.sub('_', (raw or '').strip().replace(' ', '_'))
    return re.sub(r'_+', '_', cleaned).strip('_')


def build_folder_name(experiment_id: int, suffix: str,
                      when: Optional[datetime] = None) -> str:
    stamp = (when or datetime.now()).strftime('%Y%m%d_%H%M%S')
    parts = [stamp, '{:03d}'.format(experiment_id)]
    clean = sanitise_suffix(suffix)
    if clean:
        parts.append(clean)
    return '_'.join(parts)


def scan_next_experiment_id(save_dir: str) -> Optional[int]:
    """Highest ID already on disk plus one, so a restart never collides."""
    if not os.path.isdir(save_dir):
        return None
    highest = 0
    try:
        for name in os.listdir(save_dir):
            match = _FOLDER.match(name)
            if match:
                highest = max(highest, int(match.group(1)))
    except OSError:
        return None
    return highest + 1 if highest else None
