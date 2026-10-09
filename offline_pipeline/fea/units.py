# Tier: D
"""Single SI <-> CalculiX unit conversion module (WP-7 Step A.2).

CalculiX unit system: length mm, force N, stress MPa, mass tonne, time s.
SI in this project: m, N, Pa, kg, s. Every conversion is a decimal power of ten,
so CalculiX value = SI value x 10^k with k from `_EXPONENT` below.

Why one multiply or divide: 10.0**n is an exact double for 0 <= n <= 22, so each
conversion is one correctly rounded operation. Do not chain conversions.

Valid input: finite int or float (bool rejected), any sign. Raises ValueError otherwise.
"""
import math

# quantity -> k, with CalculiX value = SI value x 10^k.
_EXPONENT: dict[str, int] = {
    "length": 3,  # 1 m = 1e3 mm
    "area": 6,  # 1 m^2 = 1e6 mm^2
    "second_moment_of_area": 12,  # 1 m^4 = 1e12 mm^4
    "force": 0,  # N in both systems
    "stress": -6,  # 1 Pa = 1e-6 MPa (also Young's modulus)
    "mass": -3,  # 1 kg = 1e-3 tonne
    "density": -12,  # kg/m^3 -> tonne/mm^3: 1e-3 / 1e9
    "time": 0,  # s in both systems
    "acceleration": 3,  # 1 m/s^2 = 1e3 mm/s^2; check N = tonne*mm/s^2 = 1e3 kg * 1e-3 m/s^2
}


def _check(value: float, quantity: str) -> int:
    """Validate input and return the exponent k for `quantity`."""
    if quantity not in _EXPONENT:
        raise ValueError(f"unknown quantity {quantity!r}; valid: {sorted(_EXPONENT)}")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"value must be int or float, got {type(value).__name__}")
    if not math.isfinite(value):
        raise ValueError(f"value must be finite, got {value}")
    return _EXPONENT[quantity]


def _scale(value: float, k: int) -> float:
    """Return value x 10^k as one multiply (k >= 0) or one divide (k < 0)."""
    result = float(value) * 10.0**k if k >= 0 else float(value) / 10.0 ** (-k)
    if not math.isfinite(result):
        raise ValueError(f"conversion overflowed: {value} with exponent {k}")
    return result


def to_calculix(value: float, quantity: str) -> float:
    """Convert an SI value (m, N, Pa, kg, s based) to mm/N/MPa/tonne/s."""
    return _scale(value, _check(value, quantity))


def from_calculix(value: float, quantity: str) -> float:
    """Convert a mm/N/MPa/tonne/s value back to SI."""
    return _scale(value, -_check(value, quantity))
