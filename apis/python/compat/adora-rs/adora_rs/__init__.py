"""Deprecated — renamed to ``dora-rs`` for 1.0. Install ``dora-rs`` instead.

This package is a compatibility shim for users who previously installed
``adora-rs`` (fork-era versions 0.1.0 and 0.2.1 on PyPI). It pulls in
``dora-rs>=1.0.0`` and re-exports its public API so existing imports
continue to work during migration.

Uninstall with ``pip uninstall adora-rs`` and install ``dora-rs`` directly
for the canonical 1.0 package.

Migration guide: https://github.com/dora-rs/dora/blob/main/docs/migration-from-0.x.md
"""

import warnings

warnings.warn(
    "adora-rs is deprecated — the project was renamed to dora-rs for 1.0. "
    "Uninstall adora-rs and install dora-rs. "
    "See https://github.com/dora-rs/dora/blob/main/docs/migration-from-0.x.md",
    DeprecationWarning,
    stacklevel=2,
)

from dora import *  # noqa: F401, F403, E402
