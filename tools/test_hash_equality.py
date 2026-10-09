# Tier: F
import hashlib
import os
import unittest

SAMPLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hash_sample.json")

# Reference: GNU coreutils `sha256sum tools/hash_sample.json` on the exact 139 bytes
# (LF endings, UTF-8). SHA-256 cannot be hand-computed; coreutils is independent of
# both Rust (sha2 in build.rs) and Python (hashlib). The Rust twin test is
# surrogate::tests::test_build_rs_sha256_matches_reference_sample.
REF = "9a9e1c38f305cf448b252551f0ad5ad19f25cf4023680f1f343083d1d93b5f5b"


class HashEquality(unittest.TestCase):
    def test_python_sha256_matches_reference_sample(self):
        # Same method as train_aero.py / train_fea.py: raw bytes, hexdigest.
        with open(SAMPLE, "rb") as f:
            data = f.read()
        self.assertEqual(len(data), 139)
        self.assertEqual(hashlib.sha256(data).hexdigest(), REF)


if __name__ == "__main__":
    unittest.main()
