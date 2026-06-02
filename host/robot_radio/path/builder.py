"""PathBuilder protocol and build_path registry.

To add a new builder, create a module that defines a class (or callable)
conforming to the ``PathBuilder`` protocol, then register it::

    from robot_radio.path.builder import _REGISTRY
    _REGISTRY["my_method"] = MyBuilder()

Or call ``build_path`` directly with the method name once the import has
been done.  The registry is populated at import time by each builder module.
"""

from __future__ import annotations

from typing import Any, Protocol

from robot_radio.path.sampled_path import SampledPath
from robot_radio.nav.pose import Pose, Waypoint


class PathBuilder(Protocol):
    """Structural protocol for path builders.

    Any object with a ``__call__`` method matching this signature is a
    valid ``PathBuilder`` — no inheritance required.
    """

    def __call__(
        self,
        start: Pose,
        end: Pose,
        waypoints: list[Waypoint],
        **kwargs: Any,
    ) -> SampledPath:
        """Build a sampled path from *start* to *end* via *waypoints*.

        Parameters
        ----------
        start:
            Starting pose (position + heading).
        end:
            Ending pose (position + heading).
        waypoints:
            Intermediate waypoints.  May be empty.  Headings may be
            ``None`` — the builder is responsible for inference.
        **kwargs:
            Builder-specific keyword arguments (e.g. ``spacing_cm``,
            ``tangent_frac``).
        """
        ...


# Registry: method name → PathBuilder instance.
# Populated by each builder module on import.
_REGISTRY: dict[str, PathBuilder] = {}


def build_path(
    method: str,
    start: Pose,
    end: Pose,
    waypoints: list[Waypoint] | None = None,
    **kwargs: Any,
) -> SampledPath:
    """Build a path using a registered builder.

    Parameters
    ----------
    method:
        Builder name to look up in the registry (e.g. ``"bezier"``).
    start:
        Starting pose.
    end:
        Ending pose.
    waypoints:
        Optional intermediate waypoints.  Defaults to an empty list.
    **kwargs:
        Forwarded verbatim to the builder.

    Returns
    -------
    SampledPath
        The sampled polyline and associated metadata.

    Raises
    ------
    KeyError
        If *method* is not found in the registry.
    """
    if method not in _REGISTRY:
        available = ", ".join(sorted(_REGISTRY)) or "(none registered)"
        raise KeyError(
            f"Unknown path builder {method!r}. Available: {available}"
        )
    return _REGISTRY[method](start, end, waypoints or [], **kwargs)
