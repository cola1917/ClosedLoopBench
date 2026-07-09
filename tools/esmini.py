from __future__ import annotations

import os
import shutil
from pathlib import Path


def find_esmini() -> Path | None:
    env_value = os.environ.get("ESMINI_BIN")
    if env_value:
        return Path(env_value)

    project_root = Path(__file__).resolve().parents[1]
    local_candidates = (
        project_root / "tools" / "esmini" / "bin" / "esmini.exe",
        project_root / "tools" / "esmini" / "dist" / "esmini" / "bin" / "esmini.exe",
    )
    for local in local_candidates:
        if local.exists():
            return local

    found = shutil.which("esmini") or shutil.which("esmini.exe")
    return Path(found) if found else None
