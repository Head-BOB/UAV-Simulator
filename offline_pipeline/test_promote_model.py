# Tier: D
"""Tests for promote_model.py. Fake repo in a temp dir; tiny real ONNX files (onnx is in the hashed lock).

Run from the repo root inside .venv:
  python -m unittest discover -s offline_pipeline -p 'test_promote_model.py'
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import promote_model as pm  # noqa: E402

# FIPS 180-4 example: SHA-256("abc") (independent reference, not computed by the code under test).
SHA256_ABC = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"


def write_onnx(path: Path, kind: str, geo: str, cfg: str) -> None:
    import onnx
    from onnx import TensorProto, helper

    x = helper.make_tensor_value_info("x", TensorProto.FLOAT, [1, 1])
    y = helper.make_tensor_value_info("y", TensorProto.FLOAT, [1, 1])
    graph = helper.make_graph([helper.make_node("Identity", ["x"], ["y"])], "g", [x], [y])
    model = helper.make_model(graph, producer_name="test")
    model.metadata_props.add(key="kind", value=kind)
    model.metadata_props.add(key="geometry_hash", value=geo)
    model.metadata_props.add(key="config_hash", value=cfg)
    path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(path))


class Repo:
    """A temp repo root with geometry/config files and two valid stub models."""

    def __init__(self, tmp: str):
        self.root = Path(tmp)
        (self.root / pm.GEOMETRY_REL).parent.mkdir(parents=True)
        (self.root / pm.GEOMETRY_REL).write_bytes(b'{"design_revision": 1}\n')
        (self.root / pm.CONFIG_REL).write_bytes(b'{"mass_kg": 1.5}\n')
        self.geo = pm.sha256_file(self.root / pm.GEOMETRY_REL)
        self.cfg = pm.sha256_file(self.root / pm.CONFIG_REL)
        for role in pm.ROLES:
            write_onnx(self.model(role), "stub", self.geo, self.cfg)

    def model(self, role: str) -> Path:
        return self.root / pm.MODEL_DIR / f"{role}_stub.onnx"

    def promote(self, role: str, **kw) -> dict:
        return pm.build_entry(self.model(role), self.root, pm.read_onnx_metadata,
                              kw.get("allow_stub", True), kw.get("holdout_error"))

    def write_manifest(self, models: dict | None = None) -> None:
        models = models or {r: self.promote(r) for r in pm.ROLES}
        manifest = {"schema_version": 1, "models": models}
        (self.root / pm.MANIFEST_REL).write_text(pm.canonical_json(manifest), encoding="utf-8")

    def check(self) -> list:
        return pm.check_file(self.root, pm.read_onnx_metadata)


class Base(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.repo = Repo(self._tmp.name)


class HashAndJson(Base):
    def test_sha256_known_vector(self):
        p = self.repo.root / "abc.bin"
        p.write_bytes(b"abc")
        self.assertEqual(pm.sha256_file(p), SHA256_ABC)

    def test_canonical_json_form(self):
        self.assertEqual(pm.canonical_json({"b": 1, "a": None}), '{\n  "a": null,\n  "b": 1\n}\n')

    def test_canonical_json_rejects_nan(self):
        with self.assertRaises(ValueError):
            pm.canonical_json({"a": float("nan")})


class PathRules(Base):
    def test_valid_path(self):
        self.assertIsNone(pm.check_path_field("offline_pipeline/trained_models/a.onnx"))

    def test_bad_paths(self):
        for bad in ["/etc/a.onnx", "offline_pipeline/trained_models/../x.onnx", "models/a.onnx",
                    "offline_pipeline/trained_models/a.txt", "offline_pipeline\\trained_models\\a.onnx",
                    "", None, 5]:
            with self.subTest(bad=bad):
                self.assertIsNotNone(pm.check_path_field(bad))


class EntryFields(Base):
    def good(self, **over) -> dict:
        e = {"path": "offline_pipeline/trained_models/a.onnx", "sha256": "a" * 64, "kind": "trained",
             "geometry_hash": "b" * 64, "config_hash": "c" * 64, "holdout_error": 0.05}
        e.update(over)
        return e

    def test_valid_trained_and_stub(self):
        self.assertEqual(pm.check_entry_fields(self.good()), [])
        self.assertEqual(pm.check_entry_fields(self.good(kind="stub", holdout_error=None)), [])

    def test_stub_with_holdout_rejected(self):
        self.assertEqual(len(pm.check_entry_fields(self.good(kind="stub"))), 1)

    def test_trained_holdout_rules(self):
        for bad in [None, -0.1, float("nan"), float("inf"), True, "0.1"]:
            with self.subTest(bad=bad):
                self.assertEqual(len(pm.check_entry_fields(self.good(holdout_error=bad))), 1)
        self.assertEqual(pm.check_entry_fields(self.good(holdout_error=0)), [])

    def test_unknown_kind_rejected(self):
        self.assertEqual(len(pm.check_entry_fields(self.good(kind="validated"))), 1)

    def test_bad_hashes_rejected(self):
        for key in ("sha256", "geometry_hash", "config_hash"):
            for bad in ["A" * 64, "a" * 63, "g" * 64, 5]:
                with self.subTest(key=key, bad=bad):
                    self.assertEqual(len(pm.check_entry_fields(self.good(**{key: bad}))), 1)

    def test_extra_and_missing_keys_rejected(self):
        e = self.good()
        e["extra"] = 1
        self.assertEqual(len(pm.check_entry_fields(e)), 1)
        del e["extra"], e["kind"]
        self.assertEqual(len(pm.check_entry_fields(e)), 1)

    def test_not_an_object(self):
        self.assertEqual(pm.check_entry_fields([]), ["entry must be an object"])


class BuildEntry(Base):
    def test_stub_needs_flag(self):
        with self.assertRaises(ValueError):
            self.repo.promote("aero", allow_stub=False)

    def test_stub_entry_values(self):
        e = self.repo.promote("aero")
        self.assertEqual(e["path"], "offline_pipeline/trained_models/aero_stub.onnx")
        self.assertEqual(e["kind"], "stub")
        self.assertEqual(e["sha256"], pm.sha256_file(self.repo.model("aero")))
        self.assertEqual((e["geometry_hash"], e["config_hash"]), (self.repo.geo, self.repo.cfg))
        self.assertIsNone(e["holdout_error"])

    def test_stub_with_holdout_rejected(self):
        with self.assertRaises(ValueError):
            self.repo.promote("aero", holdout_error=0.1)

    def test_trained_needs_holdout(self):
        write_onnx(self.repo.model("aero"), "trained", self.repo.geo, self.repo.cfg)
        with self.assertRaises(ValueError):
            self.repo.promote("aero")
        self.assertEqual(self.repo.promote("aero", holdout_error=0.02)["holdout_error"], 0.02)

    def test_model_outside_trained_models_rejected(self):
        p = self.repo.root / "elsewhere" / "m.onnx"
        write_onnx(p, "stub", self.repo.geo, self.repo.cfg)
        with self.assertRaises(ValueError):
            pm.build_entry(p, self.repo.root, pm.read_onnx_metadata, True, None)

    def test_model_outside_repo_rejected(self):
        with tempfile.TemporaryDirectory() as other:
            p = Path(other) / "m.onnx"
            write_onnx(p, "stub", self.repo.geo, self.repo.cfg)
            with self.assertRaises(ValueError):
                pm.build_entry(p, self.repo.root, pm.read_onnx_metadata, True, None)

    def test_missing_metadata_rejected(self):
        with self.assertRaises(ValueError):
            pm.build_entry(self.repo.model("aero"), self.repo.root, lambda _p: {"kind": "stub"}, True, None)


class CheckManifest(Base):
    def test_valid_manifest(self):
        self.repo.write_manifest()
        self.assertEqual(self.repo.check(), [])

    def test_missing_manifest(self):
        self.assertEqual(len(self.repo.check()), 1)

    def test_invalid_json(self):
        (self.repo.root / pm.MANIFEST_REL).write_text("{", encoding="utf-8")
        self.assertEqual(len(self.repo.check()), 1)

    def test_model_bytes_changed(self):
        self.repo.write_manifest()
        with open(self.repo.model("fea"), "ab") as f:
            f.write(b"\0")
        errs = self.repo.check()
        self.assertTrue(any("fea: sha256" in e for e in errs), errs)

    def test_corrupt_model_reports_instead_of_crashing(self):
        self.repo.write_manifest()
        self.repo.model("fea").write_bytes(b"not an onnx file")
        errs = self.repo.check()
        self.assertTrue(any("fea: sha256" in e for e in errs), errs)
        self.assertTrue(any("fea: cannot read ONNX metadata" in e for e in errs), errs)

    def test_missing_model_file(self):
        self.repo.write_manifest()
        self.repo.model("aero").unlink()
        self.assertEqual(self.repo.check(), ["aero: model file offline_pipeline/trained_models/aero_stub.onnx does not exist"])

    def test_metadata_kind_differs_from_manifest(self):
        entry = self.repo.promote("aero")
        write_onnx(self.repo.model("aero"), "trained", self.repo.geo, self.repo.cfg)
        entry["sha256"] = pm.sha256_file(self.repo.model("aero"))  # hash fixed up; only kind disagrees
        self.repo.write_manifest({"aero": entry, "fea": self.repo.promote("fea")})
        errs = self.repo.check()
        self.assertEqual(len(errs), 1)
        self.assertIn("ONNX metadata kind='trained'", errs[0])

    def test_geometry_changed_after_training(self):
        self.repo.write_manifest()
        (self.repo.root / pm.GEOMETRY_REL).write_bytes(b'{"design_revision": 2}\n')
        errs = self.repo.check()
        self.assertEqual(len(errs), 2)  # one per role
        self.assertTrue(all("geometry_hash differs from the current" in e for e in errs))

    def test_config_changed_after_training(self):
        self.repo.write_manifest()
        (self.repo.root / pm.CONFIG_REL).write_bytes(b'{"mass_kg": 2.0}\n')
        errs = self.repo.check()
        self.assertEqual(len(errs), 2)
        self.assertTrue(all("config_hash differs from the current" in e for e in errs))

    def test_non_canonical_text(self):
        self.repo.write_manifest()
        p = self.repo.root / pm.MANIFEST_REL
        p.write_text(json.dumps(json.loads(p.read_text()), indent=4), encoding="utf-8")
        errs = self.repo.check()
        self.assertEqual(len(errs), 1)
        self.assertIn("canonical", errs[0])

    def test_wrong_roles_schema_and_keys(self):
        entry = self.repo.promote("aero")
        cases = [
            ({"schema_version": 1, "models": {"aero": entry}}, "roles"),
            ({"schema_version": 2, "models": {}}, "schema_version"),
            ({"schema_version": True, "models": {}}, "schema_version"),
            ({"models": {}}, "keys"),
            ([], "keys"),
        ]
        for manifest, word in cases:
            with self.subTest(word=word, manifest=manifest):
                errs = pm.check_manifest(manifest, self.repo.root, pm.read_onnx_metadata)
                self.assertEqual(len(errs), 1)
                self.assertIn(word, errs[0])


class Main(Base):
    def run_main(self, *args) -> int:
        return pm.main(["--repo-root", str(self.repo.root), *args])

    def test_promote_then_check(self):
        for role in pm.ROLES:
            self.assertEqual(self.run_main("--role", role, "--model", str(self.repo.model(role)), "--allow-stub"), 0)
        self.assertEqual(self.run_main("--check"), 0)
        data = json.loads((self.repo.root / pm.MANIFEST_REL).read_text())
        self.assertEqual(sorted(data["models"]), ["aero", "fea"])

    def test_stub_without_flag_exits_1_and_writes_nothing(self):
        rc = self.run_main("--role", "aero", "--model", str(self.repo.model("aero")))
        self.assertEqual(rc, 1)
        self.assertFalse((self.repo.root / pm.MANIFEST_REL).exists())

    def test_stale_geometry_refused(self):
        (self.repo.root / pm.GEOMETRY_REL).write_bytes(b'{"design_revision": 9}\n')
        rc = self.run_main("--role", "aero", "--model", str(self.repo.model("aero")), "--allow-stub")
        self.assertEqual(rc, 1)
        self.assertFalse((self.repo.root / pm.MANIFEST_REL).exists())

    def test_check_with_one_role_only_fails(self):
        self.run_main("--role", "aero", "--model", str(self.repo.model("aero")), "--allow-stub")
        self.assertEqual(self.run_main("--check"), 1)

    def test_written_manifest_is_lf(self):
        for role in pm.ROLES:
            self.run_main("--role", role, "--model", str(self.repo.model(role)), "--allow-stub")
        self.assertNotIn(b"\r", (self.repo.root / pm.MANIFEST_REL).read_bytes())


if __name__ == "__main__":
    unittest.main()
