"""Locate writable resources in a relocated source workspace."""
import os
from pathlib import Path


def workspace_path(relative):
    """Resolve source assets; preserve trailing slashes used by legacy callers."""
    configured = os.environ.get('ARMPI_WS')
    if configured:
        root = Path(configured).expanduser().resolve()
        if not (root / 'src').is_dir():
            raise RuntimeError(f'ARMPI_WS has no src directory: {root}')
    else:
        root = next((parent for parent in Path(__file__).resolve().parents
                     if (parent / 'src/driver/sdk/package.xml').is_file()), None)
        if root is None:
            raise RuntimeError('Source resources require ARMPI_WS; source scripts/setup_pc.bash')
    path = str(root / relative)
    return path + '/' if relative.endswith('/') else path
