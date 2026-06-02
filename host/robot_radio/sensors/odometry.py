"""Odometry — camera-based robot pose reader with optional OTOS fallback.

Usage pattern::

    odom = Odometry(playfield, robot_tag=1)
    odom.update()           # pull one frame from the playfield
    if odom.is_valid:
        print(odom.x, odom.y, odom.yaw)

Call ``update()`` once per control loop iteration to refresh the pose.
If the robot tag is absent or stale (age > 0.3 s), ``is_valid`` is
``False`` and ``.x``, ``.y``, ``.yaw`` return ``None``.

The ``update()`` method accepts an optional *tags* argument.  Pass an
explicit ``list[Tag]`` (e.g. one yielded by ``field.stream()``) to
avoid consuming a second camera frame.  If *tags* is ``None``, the
method calls ``next()`` on ``field.stream()`` internally.

OTOS fallback
-------------
Pass an :class:`~robot_radio.sensors.otos.Otos` instance and a
:class:`~robot_radio.nav_params.NavParams` reference to enable
dead-reckoning during brief camera occlusions.

When ``params.otos_fallback_enabled`` is ``True`` and the camera tag
has been absent for longer than ``STALE_AGE`` but shorter than
``params.otos_fallback_max_age_s``, the pose is read from the OTOS
sensor instead.  Whenever a fresh camera pose is obtained, the OTOS
frame is re-aligned via ``otos.align_to(camera_pose)``.

The ``source`` attribute reports which data source is active:
``"camera"``, ``"otos"``, or ``""`` (invalid / not yet updated).

Staleness threshold
-------------------
A tag is considered fresh when ``tag.age <= 0.3`` seconds.  Tags older
than this were not seen in the most-recent camera frame and their
positions may be extrapolated or stale.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from aprilcam import Playfield, Tag
    from robot_radio.sensors.otos import Otos
    from robot_radio.nav.nav_params import NavParams

STALE_AGE = 0.3  # seconds — tags older than this are rejected


class Odometry:
    """Encapsulates robot pose reading from an AprilTag, with optional OTOS fallback.

    Parameters
    ----------
    playfield:
        A started ``aprilcam.Playfield`` instance.
    robot_tag:
        AprilTag ID that is mounted on the robot (default 1).
    otos:
        Optional :class:`~robot_radio.sensors.otos.Otos` instance.  When provided
        and ``params.otos_fallback_enabled`` is ``True``, OTOS is used as
        a dead-reckoning fallback when the camera tag is missing.
    params:
        Optional :class:`~robot_radio.nav_params.NavParams` instance used
        to read ``otos_fallback_enabled`` and ``otos_fallback_max_age_s``
        dynamically each ``update()`` call.

    After construction call :meth:`update` to populate the pose.
    Query :attr:`is_valid` before reading :attr:`x`, :attr:`y`,
    :attr:`yaw`.
    """

    def __init__(
        self,
        playfield: "Playfield",
        robot_tag: int = 1,
        otos: "Otos | None" = None,
        params: "NavParams | None" = None,
    ) -> None:
        self._field = playfield
        self._robot_tag = robot_tag
        self._otos = otos
        self._params = params
        self._stream = None  # lazy — created on first update() call

        self._x: float | None = None
        self._y: float | None = None
        self._yaw: float | None = None
        self._valid: bool = False
        self._source: str = ""

        # Timestamp of the last successful camera fix (monotonic seconds)
        self._last_camera_time: float | None = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def update(self, tags: "list[Tag] | None" = None) -> None:
        """Refresh pose from the camera (and OTOS fallback if configured).

        Parameters
        ----------
        tags:
            An explicit tag list (e.g. from ``field.stream()``).
            When *None* (the default), one frame is pulled from the
            playfield's internal stream.
        """
        if tags is None:
            if self._stream is None:
                self._stream = self._field.stream()
            try:
                tags = next(self._stream)
            except StopIteration:
                self._stream = self._field.stream()
                try:
                    tags = next(self._stream)
                except StopIteration:
                    self._handle_no_camera()
                    return

        self._apply(tags)

    @property
    def x(self) -> float | None:
        """World X coordinate in cm, or ``None`` if not valid."""
        return self._x

    @property
    def y(self) -> float | None:
        """World Y coordinate in cm, or ``None`` if not valid."""
        return self._y

    @property
    def yaw(self) -> float | None:
        """Heading in radians, or ``None`` if not valid."""
        return self._yaw

    @property
    def is_valid(self) -> bool:
        """``True`` when the last :meth:`update` found a usable pose."""
        return self._valid

    @property
    def source(self) -> str:
        """Active pose source: ``"camera"``, ``"otos"``, or ``""``."""
        return self._source

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _apply(self, tags: "list[Tag]") -> None:
        """Extract the robot tag from *tags* and store the pose.

        If a fresh camera tag is found, the pose is stored and the OTOS
        frame is re-aligned (snap-to-camera correction).

        If no fresh tag is found, falls back to OTOS or invalidates.
        """
        from robot_radio.nav.pose import Pose

        for t in tags:
            if t.id == self._robot_tag and t.wx is not None:
                if t.age > STALE_AGE:
                    continue
                # Fresh camera pose found.
                # aprilcam.Tag.orientation is now world-CCW-positive
                # (verified empirically 2026-05-28 after the daemon's
                # convention change). Match it directly — no negation.
                self._x = t.wx
                self._y = t.wy
                self._yaw = t.orientation
                self._valid = True
                self._source = "camera"
                self._last_camera_time = time.monotonic()

                # Snap-to-camera correction: keep OTOS aligned to world frame
                if self._otos is not None:
                    camera_pose = Pose(t.wx, t.wy, t.orientation)
                    self._otos.align_to(camera_pose)

                return

        # No fresh camera tag — try OTOS fallback
        self._handle_no_camera()

    def _handle_no_camera(self) -> None:
        """Handle the case where no fresh camera tag is available."""
        # Check whether fallback is enabled and OTOS is wired
        enabled = (
            self._otos is not None
            and self._params is not None
            and self._params.otos_fallback_enabled
        )

        if enabled:
            max_age = self._params.otos_fallback_max_age_s  # type: ignore[union-attr]
            now = time.monotonic()
            within_window = (
                self._last_camera_time is not None
                and (now - self._last_camera_time) <= max_age
            )
            if within_window:
                pose = self._otos.read_world_pose()  # type: ignore[union-attr]
                if pose is not None:
                    self._x = pose.x
                    self._y = pose.y
                    self._yaw = pose.heading
                    self._valid = True
                    self._source = "otos"
                    return

        self._invalidate()

    def _invalidate(self) -> None:
        self._x = None
        self._y = None
        self._yaw = None
        self._valid = False
        self._source = ""
