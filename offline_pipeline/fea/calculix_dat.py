# Tier: D
"""Parser for CalculiX `.dat` result text (WP-7 Step C.4): displacement and stress blocks.

Assumed layout (from the CalculiX documentation of *NODE PRINT / *EL PRINT output):

     displacements (vx,vy,vz) for set NALL and time  0.1000000E+01
    <blank line>
             1  0.000000E+00  0.000000E+00  0.000000E+00
     stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set EALL and time  0.1000000E+01
    <blank line>
             1   1  1.000000E+02 ...

TODO(HUMAN-VERIFY): no real CalculiX output sample is available (B-9). This layout is
written from the documented format and must be checked against a real `.dat` file.
Not covered: `.frd`, eigenvalue/frequency tables, other result keywords.

Units: values are returned exactly as written, in the model's units (mm, MPa, mm/s).
Convert with `units.py`; nothing is converted here.

Valid input: text with at least one supported block. Anything else raises ValueError
with the line number. NaN, Inf and Fortran exponents without 'E' are rejected.
"""
import math
import re
from dataclasses import dataclass

# name -> (number of integer key columns, value column names)
_BLOCKS: dict[str, tuple[int, tuple[str, ...]]] = {
    "displacements": (1, ("vx", "vy", "vz")),
    "stresses": (2, ("sxx", "syy", "szz", "sxy", "sxz", "syz")),
}
_HEADER = re.compile(
    r"^\s*([A-Za-z]+)\s*\(([^)]*)\)\s+for set\s+(\S+)\s+and time\s+(\S+)\s*$"
)
_NUMBER = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$")


@dataclass(frozen=True)
class DatBlock:
    """One result block: `rows` maps the integer key tuple to the value tuple."""

    name: str
    set_name: str
    time: float
    columns: tuple[str, ...]
    rows: dict[tuple[int, ...], tuple[float, ...]]


def _float(token: str, lineno: int) -> float:
    if not _NUMBER.match(token):
        raise ValueError(f"line {lineno}: not a plain decimal number: {token!r}")
    value = float(token)
    if not math.isfinite(value):
        raise ValueError(f"line {lineno}: non-finite value {token!r}")
    return value


def _key(token: str, lineno: int) -> int:
    if not re.fullmatch(r"\d+", token) or int(token) < 1:
        raise ValueError(f"line {lineno}: key must be an integer >= 1, got {token!r}")
    return int(token)


def _start_block(match: re.Match, lineno: int) -> DatBlock:
    name = match.group(1)
    if name not in _BLOCKS:
        raise ValueError(f"line {lineno}: unsupported block {name!r}")
    time = _float(match.group(4), lineno)
    return DatBlock(name, match.group(3), time, _BLOCKS[name][1], {})


def _add_row(block: DatBlock, tokens: list[str], lineno: int) -> None:
    n_key, columns = _BLOCKS[block.name]
    if len(tokens) != n_key + len(columns):
        raise ValueError(
            f"line {lineno}: expected {n_key + len(columns)} columns, got {len(tokens)}"
        )
    key = tuple(_key(t, lineno) for t in tokens[:n_key])
    if key in block.rows:
        raise ValueError(f"line {lineno}: duplicate key {key}")
    block.rows[key] = tuple(_float(t, lineno) for t in tokens[n_key:])


def parse_dat(text: str) -> list[DatBlock]:
    """Parse `.dat` text into blocks in file order. Raises ValueError on any anomaly."""
    blocks: list[DatBlock] = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        match = _HEADER.match(line)
        if match:
            blocks.append(_start_block(match, lineno))
        elif not blocks:
            raise ValueError(f"line {lineno}: data before any block header")
        else:
            _add_row(blocks[-1], line.split(), lineno)
    if not blocks:
        raise ValueError("no result blocks found")
    return blocks
