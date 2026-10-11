# Tier: D
"""Tests for fetch_source with an injected opener: no network is used."""
import hashlib
import json
import unittest

import fetch_source as FS
import partsdb as P
from testsupport import TempDb

URL = "https://example.com/datasheet.pdf"


def opener_of(data: bytes, status: int = 200):
    calls = []

    def op(url):
        calls.append(url)
        return data, status

    op.calls = calls
    return op


class HostTests(unittest.TestCase):
    def test_host_allowlist(self):
        allowed = ["example.com", "Maker.org"]
        self.assertTrue(FS.host_allowed("https://example.com/a", allowed))
        self.assertTrue(FS.host_allowed("https://www.example.com/a", allowed))  # subdomain
        self.assertTrue(FS.host_allowed("https://maker.org/a", allowed))        # case-insensitive
        self.assertFalse(FS.host_allowed("https://notexample.com/a", allowed))  # suffix must be a dot boundary
        self.assertFalse(FS.host_allowed("https://example.com.evil.io/a", allowed))
        self.assertFalse(FS.host_allowed("ftp://example.com/a", allowed))
        self.assertFalse(FS.host_allowed("file:///etc/passwd", allowed))
        self.assertFalse(FS.host_allowed("https://shop.example.net/a", allowed))

    def test_real_allowlist_has_no_retailer_hosts(self):
        hosts = json.loads((P.DEFAULT_DB / "allowed_sources.json").read_text(encoding="utf-8"))["hosts"]
        for retailer in ("amazon", "ebay", "aliexpress", "banggood", "getfpv", "readymaderc"):
            self.assertFalse(any(retailer in h for h in hosts), retailer)


class RedirectTests(unittest.TestCase):
    def _hop(self, allowed, newurl):
        import urllib.request
        h = FS.AllowlistRedirectHandler(allowed)
        req = urllib.request.Request("https://example.com/a")
        return h.redirect_request(req, None, 301, "Moved", {}, newurl)

    def test_redirect_to_allowed_host_is_followed(self):
        self.assertIsNotNone(self._hop(["example.com", "cdn.example.org"], "https://cdn.example.org/b"))

    def test_redirect_to_unlisted_host_is_refused(self):
        with self.assertRaises(ValueError) as cm:
            self._hop(["example.com"], "https://t-hobby.com/b")
        self.assertIn("refused", str(cm.exception))

    def test_redirect_to_lookalike_host_is_refused(self):
        with self.assertRaises(ValueError):
            self._hop(["example.com"], "https://example.com.evil.io/b")

    def test_redirect_to_non_http_scheme_is_refused(self):
        with self.assertRaises(ValueError):
            self._hop(["example.com"], "file:///etc/passwd")


class FetchTests(unittest.TestCase):
    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)

    def test_pin_records_sha_size_date(self):
        data = b"datasheet bytes"
        e = FS.fetch(URL, self.db.root, "2026-10-10", "note", opener=opener_of(data))
        self.assertEqual(e["sha256"], hashlib.sha256(data).hexdigest())  # independent computation
        self.assertEqual(e["bytes"], 15)
        self.assertEqual(P.load_pins(self.db.root)[URL]["retrieved_on"], "2026-10-10")
        self.assertEqual((self.db.root / "source_cache" / e["cache_file"]).read_bytes(), data)

    def test_refuses_host_not_listed_before_any_download(self):
        op = opener_of(b"x")
        with self.assertRaises(ValueError):
            FS.fetch("https://shop.example.net/p", self.db.root, "2026-10-10", opener=op)
        self.assertEqual(op.calls, [])
        self.assertEqual(P.load_pins(self.db.root), {})

    def test_http_error_pins_nothing(self):
        with self.assertRaises(ValueError):
            FS.fetch(URL, self.db.root, "2026-10-10", opener=opener_of(b"x", 404))
        self.assertEqual(P.load_pins(self.db.root), {})

    def test_changed_content_is_refused_without_update(self):
        FS.fetch(URL, self.db.root, "2026-10-10", opener=opener_of(b"v1"))
        with self.assertRaises(ValueError):
            FS.fetch(URL, self.db.root, "2026-10-11", opener=opener_of(b"v2"))
        self.assertEqual(P.load_pins(self.db.root)[URL]["sha256"], hashlib.sha256(b"v1").hexdigest())

    def test_update_keeps_history(self):
        FS.fetch(URL, self.db.root, "2026-10-10", opener=opener_of(b"v1"))
        FS.fetch(URL, self.db.root, "2026-10-11", update=True, opener=opener_of(b"v2"))
        pin = P.load_pins(self.db.root)[URL]
        self.assertEqual(pin["sha256"], hashlib.sha256(b"v2").hexdigest())
        self.assertEqual([h["sha256"] for h in pin["history"]], [hashlib.sha256(b"v1").hexdigest()])

    def test_same_content_refetch_is_fine(self):
        FS.fetch(URL, self.db.root, "2026-10-10", opener=opener_of(b"v1"))
        e = FS.fetch(URL, self.db.root, "2026-10-12", opener=opener_of(b"v1"))
        self.assertEqual(e["retrieved_on"], "2026-10-12")
        self.assertNotIn("history", e)

    def test_oversize_refused(self):
        big = b"0" * (FS.MAX_BYTES + 1)
        with self.assertRaises(ValueError):
            FS.fetch(URL, self.db.root, "2026-10-10", opener=opener_of(big))


if __name__ == "__main__":
    unittest.main()
