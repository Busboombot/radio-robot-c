"""robot_radio.testkit.dash — reusable live matplotlib dashboard + CSV logging.

Extracted from ``tests/bench/velocity_chart.py``.  Provides a generic
multi-panel matplotlib dashboard that tools (velocity_chart, playfield_tour,
etc.) can use as a thin driver.

matplotlib imports are LAZY (inside class methods and functions) so that
``import robot_radio.testkit.dash`` works without matplotlib installed.

Usage::

    from robot_radio.testkit.dash import Dashboard

    # Define panels: list of (panel_title, y_label, [series_name, ...])
    panels = [
        ("Wheel velocity (mm/s)", "mm/s", ["vL", "vR"]),
        ("Position (mm)", "mm",   ["x",  "y"]),
    ]
    dash = Dashboard("Robot dashboard", panels)
    dash.update({"vL": 120.0, "vR": 118.0, "x": 35.0, "y": -10.0})
    dash.save_csv("/tmp/run.csv")

The Dashboard is non-interactive until ``show()`` is called (or the caller
invokes the matplotlib event loop).  Each ``update()`` call appends data
and redraws; call ``plt.pause(0.033)`` in your driver loop for interactivity.
"""

from __future__ import annotations

import csv
import os
import time
from typing import Any


class Dashboard:
    """Generic live multi-panel matplotlib dashboard with CSV logging.

    Parameters
    ----------
    title:
        Figure window title / suptitle.
    panels:
        List of ``(panel_title, y_label, series_names)`` tuples where
        ``series_names`` is a list of string keys that will appear in
        ``update()`` data dicts.  One axes panel is created per entry.
    window_s:
        Rolling time window for the x-axis (seconds, newest at right).
    """

    def __init__(
        self,
        title: str,
        panels: list[tuple[str, str, list[str]]],
        window_s: float = 8.0,
    ) -> None:
        self._title = title
        self._panels = panels
        self._window_s = window_s

        # Data buffers: series_name -> deque of (timestamp, value)
        import collections

        maxlen = int(window_s * 100)  # generous: up to 100 Hz
        self._t_buf: collections.deque = collections.deque(maxlen=maxlen)
        self._series: dict[str, collections.deque] = {}
        for _, _, names in panels:
            for name in names:
                if name not in self._series:
                    self._series[name] = collections.deque(maxlen=maxlen)

        # CSV log: list of dicts, one row per update() call.
        self._rows: list[dict[str, Any]] = []
        self._t0: float | None = None

        # Matplotlib state (created lazily on first draw call).
        self._fig: Any = None
        self._axes: list[Any] = []
        self._lines: dict[str, Any] = {}  # series_name -> Line2D
        self._initialized = False

    # ------------------------------------------------------------------ #
    # Public API                                                           #
    # ------------------------------------------------------------------ #

    def update(self, data: dict[str, float]) -> None:
        """Append a data point and redraw all panels.

        Parameters
        ----------
        data:
            Dict mapping series_name -> float value.  Keys not listed in
            any panel's series_names are silently ignored.
        """
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now

        self._t_buf.append(now)
        for name in self._series:
            val = float(data.get(name, 0.0))
            self._series[name].append(val)

        # Record for CSV.
        row: dict[str, Any] = {"t": now - self._t0}
        row.update({k: data.get(k, None) for k in self._series})
        self._rows.append(row)

        # Redraw if the figure is open.
        if self._fig is not None and self._initialized:
            self._redraw(now)

    def show(self) -> None:
        """Initialise and show the matplotlib figure (blocking in interactive mode).

        Creates the figure on first call.  Subsequent calls are no-ops.
        """
        self._ensure_fig()
        import matplotlib.pyplot as plt  # noqa: PLC0415

        plt.ion()
        plt.show(block=False)

    def draw(self) -> None:
        """Flush one frame (non-blocking).  Call in your driver loop."""
        self._ensure_fig()
        import matplotlib.pyplot as plt  # noqa: PLC0415

        if self._fig is not None:
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()

    def pause(self, interval: float = 0.033) -> None:
        """Pause for interval seconds while processing UI events."""
        import matplotlib.pyplot as plt  # noqa: PLC0415

        plt.pause(interval)

    def is_open(self) -> bool:
        """Return True if the figure window is still open."""
        if self._fig is None:
            return False
        import matplotlib.pyplot as plt  # noqa: PLC0415

        return plt.fignum_exists(self._fig.number)

    def save_csv(self, path: str) -> None:
        """Write all accumulated rows to a CSV file at path.

        Parameters
        ----------
        path:
            Output path.  Parent directories are created if needed.
        """
        if not self._rows:
            return
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        fieldnames = list(self._rows[0].keys())
        with open(path, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

    def close(self) -> None:
        """Close the matplotlib figure."""
        if self._fig is not None:
            import matplotlib.pyplot as plt  # noqa: PLC0415

            try:
                plt.close(self._fig)
            except Exception:  # noqa: BLE001
                pass
            self._fig = None
            self._initialized = False

    # ------------------------------------------------------------------ #
    # Internal helpers                                                     #
    # ------------------------------------------------------------------ #

    def _ensure_fig(self) -> None:
        """Create the matplotlib figure if it has not been created yet."""
        if self._initialized:
            return

        import matplotlib  # noqa: PLC0415
        import matplotlib.pyplot as plt  # noqa: PLC0415

        n = len(self._panels)
        self._fig, axs = plt.subplots(n, 1, figsize=(12, 3 * max(n, 1)),
                                      squeeze=False)
        axs = [axs[i][0] for i in range(n)]
        self._axes = axs

        _COLORS = [
            "deepskyblue", "tomato", "limegreen", "violet",
            "orange", "yellow", "cyan", "magenta",
        ]

        self._fig.suptitle(self._title, fontsize=12)
        for ax, (panel_title, y_label, series_names) in zip(axs, self._panels):
            ax.set_title(panel_title, fontsize=9)
            ax.set_ylabel(y_label, fontsize=8)
            ax.set_xlabel("time (s)", fontsize=8)
            ax.grid(True, alpha=0.3)
            for i, name in enumerate(series_names):
                color = _COLORS[i % len(_COLORS)]
                (line,) = ax.plot([], [], color=color, lw=1.2, label=name)
                self._lines[name] = line
            if series_names:
                ax.legend(fontsize=7, loc="upper right")

        plt.tight_layout()
        self._initialized = True

    def _redraw(self, now: float) -> None:
        """Update all line data from current buffers."""
        import numpy as np  # noqa: PLC0415

        t_list = list(self._t_buf)
        if not t_list:
            return

        t_arr = np.array(t_list)
        age = now - t_arr  # 0 = oldest, grows to the right (absolute time)

        for name, line in self._lines.items():
            vals = list(self._series[name])
            n = min(len(t_arr), len(vals))
            if n > 0:
                line.set_data(t_arr[-n:] - t_arr[0], np.array(vals[-n:]))

        # Auto-scale each axes.
        for ax in self._axes:
            ax.relim()
            ax.autoscale_view()

        self._fig.canvas.draw_idle()
