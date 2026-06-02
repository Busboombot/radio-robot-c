"""Abstract base class for path-following controllers."""

from abc import ABC, abstractmethod


class Controller(ABC):
    """Abstract base class for path-following controllers.

    All ``compute`` implementations return ``(left, right)`` motor command
    floats clamped to [-100, 100] (project motor command units).  A return
    value of ``(0.0, 0.0)`` signals that the robot has arrived at the end
    of the path.

    All coordinates are in centimetres (project convention).
    """

    @abstractmethod
    def set_path(self, path) -> None:
        """Load a new path and reset internal state.

        Parameters
        ----------
        path:
            Ordered sequence of (x, y) waypoints in centimetres.
        """
        ...

    @abstractmethod
    def compute(self, pos, yaw) -> tuple[int, int]:
        """Compute motor commands for the current robot pose.

        Parameters
        ----------
        pos:
            Current robot position ``(x, y)`` in centimetres.
        yaw:
            Robot heading in radians, standard math convention
            (0 = east, π/2 = north).

        Returns
        -------
        tuple[int, int]
            ``(left, right)`` motor commands.  Return ``(0, 0)`` when
            the robot has reached the goal.
        """
        ...

    @abstractmethod
    def is_finished(self) -> bool:
        """Return True when the robot has reached the end of the path."""
        ...

    def reset(self) -> None:
        """Reset all internal state without loading a new path.

        Default implementation is a no-op.  Subclasses may override to
        clear caches, counters, or integrators.
        """
        pass
