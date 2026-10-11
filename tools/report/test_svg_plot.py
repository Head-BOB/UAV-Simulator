# Tier: D
import unittest
import xml.etree.ElementTree as ET

import svg_plot as S


class TickTests(unittest.TestCase):
    def test_nice_ticks_by_hand(self):
        # 0..97, ~5 intervals: raw step 19.4 -> 20 -> 0, 20, ..., 100
        self.assertEqual(S.nice_ticks(0.0, 97.0), [0.0, 20.0, 40.0, 60.0, 80.0, 100.0])
        # 0.3..0.62: raw 0.064 -> 0.1 -> 0.3 ... 0.7
        self.assertEqual(S.nice_ticks(0.3, 0.62), [0.3, 0.4, 0.5, 0.6, 0.7])
        # -3..4: raw 1.4 -> 2 -> -4, -2, 0, 2, 4
        self.assertEqual(S.nice_ticks(-3.0, 4.0), [-4.0, -2.0, 0.0, 2.0, 4.0])
        # 0..230: raw step 46 -> the 5 x 10 = 50 step (not 20 or 100) -> 0, 50, ..., 250
        self.assertEqual(S.nice_ticks(0.0, 230.0), [0.0, 50.0, 100.0, 150.0, 200.0, 250.0])
        # 0..0.023: raw 0.0046 -> 0.005 -> 0, 0.005, ..., 0.025
        self.assertEqual(S.nice_ticks(0.0, 0.023), [0.0, 0.005, 0.01, 0.015, 0.02, 0.025])
        # a flat series is widened by one unit
        self.assertEqual(S.nice_ticks(5.0, 5.0)[0], 5.0)
        self.assertGreater(S.nice_ticks(5.0, 5.0)[-1], 5.0)
        with self.assertRaises(ValueError):
            S.nice_ticks(0.0, float("inf"))


class DownsampleTests(unittest.TestCase):
    def test_short_series_untouched(self):
        xs = list(range(10))
        self.assertEqual(S.downsample(xs, xs, 600), (xs, xs))

    def test_keeps_ends_and_the_extremes_and_the_limit(self):
        n = 5000
        xs = [float(i) for i in range(n)]
        ys = [0.0] * n
        ys[1234] = 9.0
        ys[3333] = -4.0
        dx, dy = S.downsample(xs, ys, 600)
        self.assertLessEqual(len(dx), 600)
        self.assertEqual((dx[0], dx[-1]), (0.0, 4999.0))
        self.assertIn(9.0, dy)
        self.assertIn(-4.0, dy)
        self.assertEqual(dx, sorted(dx))
        with self.assertRaises(ValueError):
            S.downsample([1.0], [1.0, 2.0])


class ChartTests(unittest.TestCase):
    def chart(self, **kw):
        args = dict(series=[("a", [0.0, 10.0], [0.0, 100.0])], title="T", xlabel="x", ylabel="y")
        args.update(kw)
        return S.line_chart(**args)

    def test_is_valid_xml_and_maps_the_corners(self):
        svg = self.chart()
        root = ET.fromstring(svg)
        pl = next(e for e in root.iter() if e.tag.endswith("polyline"))
        pts = [tuple(map(float, p.split(","))) for p in pl.get("points").split()]
        # plot area: left 56, top 28, width 640-56-12 = 572, height 280-28-40 = 212; ticks 0..10 and 0..100 so corners are exact
        self.assertEqual(pts[0], (56.0, 240.0))      # (0, 0) is bottom left: top + ph = 28 + 212
        self.assertEqual(pts[1], (628.0, 28.0))      # (10, 100) is top right: 56 + 572

    def test_text_is_escaped(self):
        svg = self.chart(title='<b>"x"</b> & y', series=[("<i>", [0.0, 1.0], [0.0, 1.0])], markers=[(0.5, "<m>")])
        ET.fromstring(svg)
        self.assertNotIn("<b>", svg)
        self.assertNotIn("<i>", svg)
        self.assertIn("&lt;b&gt;", svg)

    def test_empty_series_says_no_data(self):
        svg = self.chart(series=[("a", [], [])])
        ET.fromstring(svg)
        self.assertIn("no data", svg)

    def test_markers_outside_the_range_are_dropped_and_output_is_deterministic(self):
        a = self.chart(markers=[(5.0, "in"), (99.0, "out")])
        self.assertIn(">in<", a)
        self.assertNotIn(">out<", a)
        self.assertEqual(a, self.chart(markers=[(5.0, "in"), (99.0, "out")]))


if __name__ == "__main__":
    unittest.main()
