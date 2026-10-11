# Tier: D
"""Small inline-SVG line charts (no plotting library). All text is escaped; the output is deterministic."""
from __future__ import annotations

import math
from xml.sax.saxutils import escape

COLOURS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"]
MAX_POINTS = 600


def nice_ticks(lo: float, hi: float, target: int = 5) -> list[float]:
    """Round tick values covering [lo, hi]; step is 1, 2 or 5 times a power of ten."""
    if not (math.isfinite(lo) and math.isfinite(hi)):
        raise ValueError("axis range must be finite")
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / max(1, target)
    mag = 10.0 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1.0, 2.0, 5.0, 10.0) if m * mag >= raw - 1e-12 * raw)
    first = math.floor(lo / step + 1e-9) * step
    ticks, k = [], 0
    while True:
        v = round(first + k * step, 12)
        ticks.append(v)
        if v >= hi - 1e-9 * step:
            break
        k += 1
    return ticks


def downsample(xs: list[float], ys: list[float], limit: int = MAX_POINTS) -> tuple[list[float], list[float]]:
    """At most `limit` points; keeps the first, the last and the minimum and maximum of every bucket."""
    n = len(xs)
    if n != len(ys):
        raise ValueError("x and y length mismatch")
    if n <= limit:
        return list(xs), list(ys)
    buckets = max(1, limit // 2 - 1)
    keep = {0, n - 1}
    for b in range(buckets):
        lo, hi = b * n // buckets, (b + 1) * n // buckets
        if hi > lo:
            seg = range(lo, hi)
            keep.add(min(seg, key=lambda i: (ys[i], i)))
            keep.add(max(seg, key=lambda i: (ys[i], -i)))
    idx = sorted(keep)
    return [xs[i] for i in idx], [ys[i] for i in idx]


def _fmt(v: float) -> str:
    return f"{v:.2f}".rstrip("0").rstrip(".")


def line_chart(series: list[tuple[str, list[float], list[float]]], title: str, xlabel: str, ylabel: str,
               markers: list[tuple[float, str]] | None = None, width: int = 640, height: int = 280) -> str:
    """One SVG string. `series` = [(label, xs, ys)]. `markers` = vertical event lines [(x, label)]."""
    left, right, top, bottom = 56, 12, 28, 40
    pw, ph = width - left - right, height - top - bottom
    pts = [(s[0], *downsample(s[1], s[2])) for s in series if len(s[1]) > 0]
    head = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
            f'role="img" aria-label="{escape(title, {chr(34): "&quot;"})}">')
    parts = [head, f'<text x="{left}" y="16" font-size="13" font-weight="bold">{escape(title)}</text>']
    if not pts:
        parts.append(f'<text x="{width // 2}" y="{height // 2}" text-anchor="middle" font-size="12">no data</text></svg>')
        return "".join(parts)
    xs_all = [x for _, xs, _ in pts for x in xs]
    ys_all = [y for _, _, ys in pts for y in ys]
    xt, yt = nice_ticks(min(xs_all), max(xs_all)), nice_ticks(min(ys_all), max(ys_all))
    x0, x1, y0, y1 = xt[0], xt[-1], yt[0], yt[-1]

    def px(x: float) -> float:
        return left + (x - x0) / (x1 - x0) * pw

    def py(y: float) -> float:
        return top + ph - (y - y0) / (y1 - y0) * ph

    for v in yt:
        parts.append(f'<line x1="{left}" x2="{left + pw}" y1="{_fmt(py(v))}" y2="{_fmt(py(v))}" stroke="#ccc" stroke-width="0.5"/>'
                     f'<text x="{left - 4}" y="{_fmt(py(v) + 3)}" text-anchor="end" font-size="10">{escape(_fmt(v))}</text>')
    for v in xt:
        parts.append(f'<line x1="{_fmt(px(v))}" x2="{_fmt(px(v))}" y1="{top}" y2="{top + ph}" stroke="#eee" stroke-width="0.5"/>'
                     f'<text x="{_fmt(px(v))}" y="{top + ph + 14}" text-anchor="middle" font-size="10">{escape(_fmt(v))}</text>')
    parts.append(f'<rect x="{left}" y="{top}" width="{pw}" height="{ph}" fill="none" stroke="#666"/>')
    for x, label in markers or []:
        if x0 <= x <= x1:
            parts.append(f'<line x1="{_fmt(px(x))}" x2="{_fmt(px(x))}" y1="{top}" y2="{top + ph}" stroke="#888" stroke-dasharray="4 3"/>'
                         f'<text x="{_fmt(px(x) + 2)}" y="{top + 10}" font-size="9" fill="#555">{escape(label)}</text>')
    for k, (label, xs, ys) in enumerate(pts):
        d = " ".join(f"{_fmt(px(x))},{_fmt(py(y))}" for x, y in zip(xs, ys))
        c = COLOURS[k % len(COLOURS)]
        parts.append(f'<polyline fill="none" stroke="{c}" stroke-width="1.5" points="{d}"/>')
        lx = left + 8 + k * 130
        parts.append(f'<line x1="{lx}" x2="{lx + 14}" y1="{height - 8}" y2="{height - 8}" stroke="{c}" stroke-width="2"/>'
                     f'<text x="{lx + 18}" y="{height - 4}" font-size="10">{escape(label)}</text>')
    parts.append(f'<text x="{left + pw // 2}" y="{height - 20}" text-anchor="middle" font-size="10">{escape(xlabel)}</text>')
    parts.append(f'<text x="12" y="{top + ph // 2}" text-anchor="middle" font-size="10" transform="rotate(-90 12 {top + ph // 2})">'
                 f'{escape(ylabel)}</text></svg>')
    return "".join(parts)
