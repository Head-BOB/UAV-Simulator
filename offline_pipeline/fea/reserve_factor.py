# Tier: D
"""Reserve factor (WP-7 Step C.2): RF = allowable / (applied x factor of safety).

RF < 1.0 is failure. This is the number telemetry shows.
"""
import math


def _real(value: float, name: str) -> float:
    """Return value as float, rejecting bool, non-numbers and non-finite values."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be int or float, got {type(value).__name__}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return float(value)


def reserve_factor(
    allowable_stress: float, applied_stress: float, factor_of_safety: float
) -> float:
    """Return RF = allowable / (applied x FoS), dimensionless.

    allowable_stress and applied_stress must be in the SAME unit (Pa or MPa), > 0.
    applied_stress is the criterion stress magnitude (von Mises for metals).
    A zero applied stress is rejected, not returned as infinity (an unloaded case
    is for the caller to report; inf is not valid JSON in results.json).
    factor_of_safety must be >= 1.0 (a value below 1 is unconservative).
    Raises ValueError on any violation or a non-finite result.
    """
    allowable = _real(allowable_stress, "allowable_stress")
    applied = _real(applied_stress, "applied_stress")
    fos = _real(factor_of_safety, "factor_of_safety")
    if allowable <= 0.0:
        raise ValueError(f"allowable_stress must be > 0, got {allowable}")
    if applied <= 0.0:
        raise ValueError(f"applied_stress must be > 0, got {applied}")
    if fos < 1.0:
        raise ValueError(f"factor_of_safety must be >= 1.0, got {fos}")
    result = allowable / (applied * fos)
    if not math.isfinite(result):
        raise ValueError("reserve factor is not finite (overflow or underflow)")
    return result
