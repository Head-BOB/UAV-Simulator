# Tier: D
"""Time-averaged power breakdown from one time series: pack power, shaft power, and the losses split by where they occur."""
from __future__ import annotations

import math

TWO_PI_OVER_60 = 2.0 * math.pi / 60.0   # rpm -> rad/s


class PowerError(ValueError):
    """The series cannot be averaged."""


def shaft_power_w(series: dict, i: int) -> float:
    """Mechanical power at row i: sum over the four motors of torque x angular speed."""
    return sum(series[f"torque_nm_{m}"][i] * series[f"rpm_{m}"][i] * TWO_PI_OVER_60 for m in range(1, 5))


def time_average(t: list[float], y: list[float]) -> float:
    """Trapezoid mean of y over t. One row: that value. Refuses a time that does not increase."""
    if len(t) != len(y) or not t:
        raise PowerError("series length mismatch or empty")
    if len(t) == 1:
        return y[0]
    area = 0.0
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        if dt <= 0.0:
            raise PowerError("time must increase")
        area += 0.5 * (y[k] + y[k - 1]) * dt
    return area / (t[-1] - t[0])


def breakdown(series: dict) -> dict:
    """Average W and % of pack power: pack, shaft, copper, no-load, ESC, other (battery internal + anything not modelled)."""
    t = series["t_s"]
    n = len(t)
    shaft = [shaft_power_w(series, i) for i in range(n)]
    pack = series["battery_w"]
    out = {"pack_w": time_average(t, pack), "shaft_w": time_average(t, shaft),
           "motor_copper_w": time_average(t, series["motor_copper_w"]),
           "motor_noload_w": time_average(t, series["motor_noload_w"]),
           "esc_loss_w": time_average(t, series["esc_loss_w"])}
    # the remainder is what the model does not split (battery internal loss, rounding); 1e-9 W of float noise is shown as zero
    other = out["pack_w"] - out["shaft_w"] - out["motor_copper_w"] - out["motor_noload_w"] - out["esc_loss_w"]
    out["other_w"] = 0.0 if abs(other) < 1e-9 else other
    out["loss_total_w"] = out["pack_w"] - out["shaft_w"]
    p = out["pack_w"]
    out["percent_of_pack"] = {k[:-2]: (100.0 * out[k] / p if p > 0.0 else None)
                              for k in ("shaft_w", "motor_copper_w", "motor_noload_w", "esc_loss_w", "other_w", "loss_total_w")}
    out["window_s"] = [t[0], t[-1]]
    out["rows"] = n
    return out
