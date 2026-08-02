"""Small, side-effect-free process diagnostics used by the desktop wrapper."""

from __future__ import annotations

import os
import platform
import resource
import subprocess
import sys
from typing import Any


def desktop_mode_enabled() -> bool:
    return os.getenv("F1_DESKTOP_MODE", "0").lower() in {"1", "true", "yes"}


def desktop_token() -> str | None:
    token = os.getenv("F1_DESKTOP_TOKEN", "")
    return token or None


def current_rss_bytes(pid: int | None = None) -> int | None:
    """Return current RSS without making psutil a mandatory runtime dependency."""
    target_pid = int(pid or os.getpid())
    try:
        import psutil  # type: ignore[import-not-found]

        return int(psutil.Process(target_pid).memory_info().rss)
    except (ImportError, OSError, ValueError):
        pass

    if sys.platform == "darwin":
        try:
            output = subprocess.check_output(
                ["ps", "-o", "rss=", "-p", str(target_pid)],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=0.25,
            )
            return int(output.strip().splitlines()[0]) * 1024
        except (IndexError, OSError, subprocess.SubprocessError, ValueError):
            return None
    return None


def current_cpu_percent(pid: int | None = None) -> float | None:
    """Return the sidecar CPU percentage without requiring psutil."""
    target_pid = int(pid or os.getpid())
    try:
        import psutil  # type: ignore[import-not-found]

        return float(psutil.Process(target_pid).cpu_percent(interval=None))
    except (ImportError, OSError, ValueError):
        pass

    if sys.platform == "darwin":
        try:
            output = subprocess.check_output(
                ["ps", "-o", "%cpu=", "-p", str(target_pid)],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=0.25,
            )
            return float(output.strip().splitlines()[0])
        except (IndexError, OSError, subprocess.SubprocessError, ValueError):
            return None
    return None


def peak_rss_bytes() -> int:
    value = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    # macOS reports bytes, Linux reports KiB.
    return value if platform.system() == "Darwin" else value * 1024


def process_metrics() -> dict[str, Any]:
    pid = os.getpid()
    return {
        "pid": pid,
        "rss_bytes": current_rss_bytes(pid),
        "peak_rss_bytes": peak_rss_bytes(),
        "cpu_percent": current_cpu_percent(pid),
        "rss_source": "psutil-or-ps" if sys.platform == "darwin" else "unavailable",
        "cpu_source": "psutil-or-ps" if sys.platform == "darwin" else "unavailable",
    }
