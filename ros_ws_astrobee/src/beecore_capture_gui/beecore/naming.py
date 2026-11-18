"""Bag folder naming.

Layout produced per run:

    <save_dir>/v<M>_<m>_<p>/session_<SSS>/uid_<UUU>/<run>/
        <run>.bag
        SUCCESS | FAILURE
        metadata.json
        params.yaml

    <run> = <YYYYMMDDTHHMMSSZ>_s<SSS>_u<UUU>[_<suffix>]

The timestamp is ISO 8601 basic format, UTC, taken when Start is pressed.
IDs are zero-padded to three digits and simply grow past 999.
"""

import os
import re
from datetime import datetime, timezone
from typing import Optional, Tuple

_UNSAFE = re.compile(r'[^A-Za-z0-9_\-]')

STAMP_PLACEHOLDER = 'YYYYMMDDTHHMMSSZ'


def sanitise_suffix(raw: str) -> str:
    """Spaces to underscores, drop anything else awkward in a path."""
    cleaned = _UNSAFE.sub('_', (raw or '').strip().replace(' ', '_'))
    return re.sub(r'_+', '_', cleaned).strip('_')


def utc_now() -> datetime:
    return datetime.now(timezone.utc).replace(microsecond=0)


def stamp_basic(when: datetime) -> str:
    """20261118T164400Z - for folder and file names."""
    return when.strftime('%Y%m%dT%H%M%SZ')


def stamp_extended(when: datetime) -> str:
    """2026-11-18T16:44:00Z - for metadata."""
    return when.strftime('%Y-%m-%dT%H:%M:%SZ')


def version_tag(version: Tuple[int, int, int]) -> str:
    return 'v{}_{}_{}'.format(*version)


def run_name(stamp: str, session_id: int, uid: int, suffix: str) -> str:
    parts = [stamp, 's{:03d}'.format(session_id), 'u{:03d}'.format(uid)]
    clean = sanitise_suffix(suffix)
    if clean:
        parts.append(clean)
    return '_'.join(parts)


def run_dir(save_dir: str, version: Tuple[int, int, int], session_id: int,
            uid: int, suffix: str, when: Optional[datetime] = None) -> str:
    """Full run folder path. With no `when`, the timestamp is a placeholder."""
    stamp = stamp_basic(when) if when else STAMP_PLACEHOLDER
    return os.path.join(save_dir, version_tag(version),
                        'session_{:03d}'.format(session_id),
                        'uid_{:03d}'.format(uid),
                        run_name(stamp, session_id, uid, suffix))
