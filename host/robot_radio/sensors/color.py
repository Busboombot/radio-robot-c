"""Colour classification for the Nezha V2 + APDS9960-alt at I2C 0x43.

The classifier is sensor- and card-specific by design.  Different
robots will see the same physical paint with different RGBC counts,
because:

* Each sensor has its own white-balance bias.  Our Nezha's sensor
  reads more strongly in B than R over a white surface — without a
  per-sensor reference, white reads as 'cyan'.
* Different test cards use different pigments, so even after white-
  balance the hue-bucket boundaries that match the user's mental
  model differ per card.
* Brightness gates depend on ambient lighting too.

So this module exposes a generic :class:`ColorClassifier` whose state
(white reference + thresholds + hue buckets) is owned by the caller.
For the Nezha + standard test card, use :func:`nezha_classifier` to
get a pre-tuned instance.  For a different robot, build a new factory.

Typical usage::

    from robot_radio.sensors.color import nezha_classifier, calibrate_white

    clf = nezha_classifier()
    calibrate_white(clf, read_fn=lambda: _read_color(robot))
    name, hue, spread, brightness = clf.classify(r, g, b, c)
"""

from __future__ import annotations

import colorsys
import time
from dataclasses import dataclass, field
from typing import Callable, Optional


# Hue buckets — (upper_bound_deg, name).  Checked in order, exclusive
# upper bound.  Red wraps around 0° and is handled separately by the
# `red_low` / `red_high` fields below; everything else goes here.
DEFAULT_HUE_BUCKETS_NEZHA: list[tuple[float, str]] = [
    (45.0,  "orange"),
    (90.0,  "yellow"),
    (165.0, "green"),
    (200.0, "cyan"),
    (255.0, "blue"),
    (290.0, "purple"),
    (345.0, "magenta"),
]

# Default white reference (R, G, B, C) for the Nezha — typical reading
# over the start-strip white on 2026-05-11.  Used when classify() is
# called without an explicit calibrate_white() — readings will be
# rough but in the right ballpark.  Call calibrate_white() on each
# session for accuracy.
DEFAULT_WHITE_REF_NEZHA: tuple[int, int, int, int] = (2500, 5350, 6460, 14800)


@dataclass
class ColorClassifier:
    """Per-robot colour classifier — owns calibration and thresholds.

    Construct directly to tune from scratch, or use a robot-specific
    factory like :func:`nezha_classifier` for sensible defaults.
    """

    # White reference (calibrated against the start strip).
    white_r: int = 0
    white_g: int = 0
    white_b: int = 0
    white_c: int = 0

    # Brightness gates (c / c_white).
    black_brightness_max: float = 0.20
    white_brightness_min: float = 0.65
    min_brightness_for_colour: float = 0.12

    # Chromatic-spread threshold above which a reading is "colored".
    spread_threshold: float = 0.30

    # Hue bucket table (upper bound exclusive, in degrees).
    hue_buckets: list[tuple[float, str]] = field(
        default_factory=lambda: list(DEFAULT_HUE_BUCKETS_NEZHA))
    red_low: float = 12.0    # h < this is "red"
    red_high: float = 345.0  # h >= this is "red"

    # ── Calibration ──────────────────────────────────────────────────

    def is_calibrated(self) -> bool:
        return self.white_c > 0

    def set_white(self, r: int, g: int, b: int, c: int) -> None:
        """Install a white reference (raw RGBC counts over white)."""
        self.white_r, self.white_g, self.white_b, self.white_c = r, g, b, c

    # ── Classification ───────────────────────────────────────────────

    def classify(self, r: int, g: int, b: int, c: int
                 ) -> tuple[str, int, int, int]:
        """Classify a raw RGBC reading.

        Returns ``(name, hue_deg, spread_pct, brightness_pct)`` where:

        * **name** is one of: black, white, gray, red, orange, yellow,
          green, cyan, blue, purple, magenta.
        * **hue_deg** is 0..360 (0 for neutrals).
        * **spread_pct** is the normalized chromatic spread × 100.
        * **brightness_pct** is c/white_c × 100.

        Raises :class:`RuntimeError` if no white reference is set.
        """
        if not self.is_calibrated():
            raise RuntimeError(
                "ColorClassifier not calibrated. Call set_white() or "
                "calibrate_white() first.")

        c_ratio = c / self.white_c
        brightness_pct = round(c_ratio * 100)

        # White-balance: a surface that matches the white reference
        # produces (1, 1, 1) here.
        r_b = r / max(self.white_r, 1)
        g_b = g / max(self.white_g, 1)
        b_b = b / max(self.white_b, 1)
        avg = (r_b + g_b + b_b) / 3.0
        if avg <= 0:
            return ("black", 0, 0, 0)
        spread = (max(r_b, g_b, b_b) - min(r_b, g_b, b_b)) / avg

        # Strongly coloured AND bright enough to trust — classify by hue.
        if (spread > self.spread_threshold
                and c_ratio > self.min_brightness_for_colour):
            m = max(r_b, g_b, b_b)
            rr, gg, bb = r_b / m, g_b / m, b_b / m
            h, _, _ = colorsys.rgb_to_hls(rr, gg, bb)
            h_deg = h * 360.0
            return (self._bucket_for_hue(h_deg),
                    round(h_deg), round(spread * 100), brightness_pct)

        # Low spread (or too dim for chroma) — decide by brightness alone.
        if c_ratio < self.black_brightness_max:
            return ("black", 0, round(spread * 100), brightness_pct)
        if c_ratio > self.white_brightness_min:
            return ("white", 0, round(spread * 100), brightness_pct)
        return ("gray", 0, round(spread * 100), brightness_pct)

    def _bucket_for_hue(self, h_deg: float) -> str:
        if h_deg < self.red_low or h_deg >= self.red_high:
            return "red"
        for upper, name in self.hue_buckets:
            if h_deg < upper:
                return name
        return "red"  # unreached if buckets cover up to red_high


# ── Factories ────────────────────────────────────────────────────────

def nezha_classifier() -> ColorClassifier:
    """Classifier pre-tuned for the Nezha V2's APDS9960-alt sensor
    against the standard test card (2026-05-11 baseline).

    A default white reference is installed so the classifier is
    usable without an explicit calibration step — but call
    :func:`calibrate_white` once per session for accuracy.
    """
    clf = ColorClassifier()
    r, g, b, c = DEFAULT_WHITE_REF_NEZHA
    clf.set_white(r, g, b, c)
    return clf


# ── Calibration helper ───────────────────────────────────────────────

def calibrate_white(
    clf: ColorClassifier,
    read_fn: Callable[[], Optional[tuple[int, int, int, int]]],
    samples: int = 5,
    sleep_s: float = 0.05,
) -> tuple[int, int, int, int]:
    """Average several RGBC readings over a known white surface and
    install the result as ``clf``'s white reference.

    ``read_fn`` must return ``(r, g, b, c)`` (or ``None`` to skip a
    sample).  Returns the installed reference tuple.
    """
    sums = [0, 0, 0, 0]
    taken = 0
    for _ in range(samples):
        rgbc = read_fn()
        if rgbc is not None:
            for i, v in enumerate(rgbc):
                sums[i] += v
            taken += 1
        time.sleep(sleep_s)
    if taken == 0:
        raise RuntimeError("No readings captured during white calibration.")
    r, g, b, c = (s // taken for s in sums)
    clf.set_white(r, g, b, c)
    return (r, g, b, c)
