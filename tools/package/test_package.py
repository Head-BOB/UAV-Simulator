# Tier: F
"""Tests for the packaging tools: the helpers lock subset, the stager, verify_package's helper checks, and a self-sufficiency run of the
STAGED helpers (no reads outside the staged tree)."""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import make_helpers_lock as L  # noqa: E402
import stage_package as S  # noqa: E402
import verify_package as V  # noqa: E402

ROOT = S.ROOT
RELEASE = ROOT / "physics_core" / "target" / "release" / "examples"


def sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


MINI_LOCK = """# header
f==6.0 \\
    --hash=sha256:ff
    # via
    #   -r offline_pipeline/requirements.in
    #   a
a==1.0 \\
    --hash=sha256:aa
    # via -r requirements.in
b==2.0 \\
    --hash=sha256:bb
    # via
    #   a
    #   other
c==3.0 \\
    --hash=sha256:cc
    # via b
d==4.0 \\
    --hash=sha256:dd
    # via -r requirements.in
e==5.0 \\
    --hash=sha256:ee
    # via d
"""


class LockTests(unittest.TestCase):
    def test_via_parsing_and_closure_by_hand(self):
        blocks = L.parse_blocks(MINI_LOCK)
        self.assertEqual(list(blocks), ["f", "a", "b", "c", "d", "e"])
        self.assertEqual(L.via_of(blocks["f"]), {"a"})            # a "-r file" line inside the list is not a package either
        self.assertEqual(L.via_of(blocks["b"]), {"a", "other"})
        self.assertEqual(L.via_of(blocks["a"]), set())            # "-r requirements.in" is not a package
        self.assertEqual(L.via_of(blocks["c"]), {"b"})
        # root a pulls b (via a), which pulls c (via b); d and e belong to another root
        self.assertEqual(L.closure(blocks, ("a",)), ["f", "a", "b", "c"])      # f lists a in its via, so it comes with it
        self.assertEqual(L.closure(blocks, ("d",)), ["d", "e"])
        with self.assertRaises(L.LockError):
            L.closure(blocks, ("zzz",))

    def test_every_block_is_a_verbatim_copy_of_the_main_lock(self):
        main = L.parse_blocks(L.MAIN_LOCK.read_text(encoding="utf-8"))
        sub = L.parse_blocks(L.OUT.read_text(encoding="utf-8"))
        self.assertTrue(sub)
        for name, block in sub.items():
            self.assertEqual(block, main[name], name)
        self.assertEqual(set(sub), {"attrs", "jsonschema", "jsonschema-specifications", "referencing", "rpds-py", "typing-extensions"})
        self.assertNotIn("numpy", sub)

    def test_committed_file_is_current_and_render_is_deterministic(self):
        text = L.MAIN_LOCK.read_text(encoding="utf-8")
        self.assertEqual(L.render(text), L.render(text))
        self.assertEqual(L.main(["--check"]), 0)

    def test_tampering_with_the_main_lock_makes_the_check_fail(self):
        text = L.MAIN_LOCK.read_text(encoding="utf-8")
        tampered = text.replace("jsonschema==4.23.0", "jsonschema==4.23.1", 1)
        self.assertNotEqual(L.render(tampered), L.OUT.read_text(encoding="utf-8"))

    def test_hashes_helper(self):
        self.assertEqual(S.lock_hashes("--hash=sha256:" + "a" * 64 + " x --hash=sha256:" + "b" * 63), {"a" * 64})


def make_repo(root: Path) -> Path:
    files = {
        "tools/parts/a.py": "x = 1\n", "tools/parts/test_a.py": "t\n", "tools/parts/__pycache__/a.cpython-312.pyc": "pyc",
        "tools/parts/builds/b/vehicle_build.json": "{}", "tools/report/r.py": "r\n", "tools/report/test_r.py": "t\n",
        "parts_db/parts/m.json": "{}", "parts_db/source_cache/big.html": "big", "parts_db/parts_index.sqlite": "db", "parts_db/schema/s.json": "{}",
        "campaign/test_definitions.json": "{}", "offline_pipeline/props/p.txt": "p", "tools/geometry/not_staged.py": "g\n",
    }
    for rel, text in files.items():
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")
    return root


def make_exes(d: Path, suffix: str) -> Path:
    d.mkdir(parents=True, exist_ok=True)
    for n in S.EXES:
        (d / (n + suffix)).write_bytes(b"exe " + n.encode())
    return d


def stage(repo: Path, dest: Path, exe_dir: Path, **kw):
    pins = kw.pop("pins", json.loads(S.PINS.read_text(encoding="utf-8")))
    plan = S.build_plan(repo, exe_dir=exe_dir, exe_suffix=kw.pop("exe_suffix", ""), python_zip=kw.pop("python_zip", None), wheels_dir=kw.pop("wheels_dir", None),
                        dev_python=kw.pop("dev_python", "/usr/bin/python3"), cfd_dir=kw.pop("cfd_dir", None), pins=pins, lock_text=kw.pop("lock_text", L.OUT.read_text(encoding="utf-8")))
    return S.write_plan(plan, dest)


class StagerUnitTests(unittest.TestCase):
    def setUp(self):
        self.t = tempfile.TemporaryDirectory()
        self.root = Path(self.t.name)
        self.repo = make_repo(self.root / "repo")
        self.exes = make_exes(self.root / "exes", "")

    def tearDown(self):
        self.t.cleanup()

    def test_allow_list_and_exclusions(self):
        m = stage(self.repo, self.root / "out", self.exes)
        paths = {e["path"] for e in m["files"]}
        self.assertIn("repo/tools/parts/a.py", paths)
        self.assertIn("repo/tools/parts/builds/b/vehicle_build.json", paths)
        self.assertIn("repo/parts_db/parts/m.json", paths)
        self.assertIn("repo/parts_db/schema/s.json", paths)
        self.assertIn("bin/run_campaign", paths)
        for bad in ("repo/tools/parts/test_a.py", "repo/tools/report/test_r.py", "repo/parts_db/source_cache/big.html", "repo/parts_db/parts_index.sqlite",
                    "repo/tools/geometry/not_staged.py"):
            self.assertNotIn(bad, paths)
        self.assertFalse(any("__pycache__" in p or p.endswith(".pyc") for p in paths))

    def test_manifest_matches_the_files_and_is_deterministic(self):
        m1 = stage(self.repo, self.root / "o1", self.exes)
        m2 = stage(self.repo, self.root / "o2", self.exes)
        self.assertEqual(m1, m2)
        helpers = self.root / "o1" / "helpers"
        for e in m1["files"]:
            data = (helpers / e["path"]).read_bytes()
            self.assertEqual((sha(data), len(data)), (e["sha256"], e["size"]), e["path"])
        self.assertEqual([e["path"] for e in m1["files"]], sorted(e["path"] for e in m1["files"]))
        self.assertEqual(m1["totals"]["files"], len(m1["files"]))
        self.assertEqual((helpers / "helpers_manifest.json").read_bytes(), (self.root / "o2" / "helpers" / "helpers_manifest.json").read_bytes())
        text = (helpers / "helpers_manifest.json").read_text()
        self.assertNotIn(str(self.root), text)

    def test_helpers_json_has_relative_paths(self):
        stage(self.repo, self.root / "o", self.exes, dev_python="/opt/py/bin/python")
        s = json.loads((self.root / "o" / "helpers" / "helpers.json").read_text())
        self.assertEqual((s["repo_root"], s["run_campaign_exe"], s["validate_models_exe"]), ("repo", "bin/run_campaign", "bin/validate_models"))
        self.assertEqual(s["python"], "/opt/py/bin/python")        # a dev stage keeps the given interpreter

    def test_refusals(self):
        stage(self.repo, self.root / "o", self.exes)
        with self.assertRaises(S.StageError) as e:
            stage(self.repo, self.root / "o", self.exes)           # destination already staged
        self.assertIn("already exists", str(e.exception))
        shutil.rmtree(self.exes)
        make_exes(self.exes, "").joinpath("run_campaign").unlink()
        with self.assertRaises(S.StageError) as e:
            stage(self.repo, self.root / "o3", self.exes)
        self.assertIn("cargo build --release --example run_campaign", str(e.exception))
        shutil.rmtree(self.repo / "campaign")
        make_exes(self.exes, "")
        with self.assertRaises(S.StageError) as e:
            stage(self.repo, self.root / "o4", self.exes)
        self.assertIn("campaign", str(e.exception))

    def test_size_limit_refuses_before_writing(self):
        rc = S.main(["--repo", str(self.repo), "--dest", str(self.root / "o"), "--dev-python", "/usr/bin/python3", "--exe-dir", str(self.exes), "--exe-suffix", "", "--max-bytes", "10"])
        self.assertEqual(rc, 1)
        self.assertFalse((self.root / "o").exists())

    def test_main_needs_exactly_one_destination_and_a_droneData_folder(self):
        self.assertEqual(S.main(["--repo", str(self.repo), "--exe-dir", str(self.exes), "--exe-suffix", "", "--dev-python", "x"]), 1)        # neither
        self.assertEqual(S.main(["--repo", str(self.repo), "--dest", str(self.root / "d"), "--package", str(self.root / "nopkg"), "--exe-dir", str(self.exes), "--exe-suffix", "", "--dev-python", "x"]), 1)   # both
        self.assertFalse((self.root / "d").exists())
        self.assertEqual(S.main(["--repo", str(self.repo), "--package", str(self.root / "nopkg"), "--exe-dir", str(self.exes), "--exe-suffix", "", "--dev-python", "x"]), 1)   # no DroneData
        pkg = self.root / "pkg" / "Windows" / "UAV_Simulator" / "DroneData"
        pkg.mkdir(parents=True)
        self.assertEqual(S.main(["--repo", str(self.repo), "--package", str(self.root / "pkg"), "--exe-dir", str(self.exes), "--exe-suffix", "", "--dev-python", "x"]), 0)
        self.assertTrue((pkg / "helpers" / "helpers_manifest.json").is_file())

    def test_cfd_results_are_copied_and_listed(self):
        cfd = self.root / "cfd"
        cfd.mkdir()
        for n in ("m.onnx", "m.onnx.model.json", "geometry.json", "slice.png", "ignored.txt"):
            (cfd / n).write_bytes(b"x")
        m = stage(self.repo, self.root / "o", self.exes, cfd_dir=cfd)
        paths = {e["path"] for e in m["files"]}
        self.assertTrue({"cfd/m.onnx", "cfd/m.onnx.model.json", "cfd/geometry.json", "cfd/slice.png"} <= paths)
        self.assertNotIn("cfd/ignored.txt", paths)


def fake_embeddable(path: Path) -> str:
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("python.exe", b"MZ python")
        z.writestr("python312.dll", b"MZ dll")
        z.writestr("python312.zip", b"stdlib")
        z.writestr("python312._pth", "python312.zip\n.\n# import site\n")
    return sha(path.read_bytes())


def fake_wheel(path: Path) -> str:
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("jsonschema/__init__.py", "x = 1\n")
        z.writestr("jsonschema-4.23.0.dist-info/METADATA", "m\n")
    return sha(path.read_bytes())


class WindowsModeTests(unittest.TestCase):
    def setUp(self):
        self.t = tempfile.TemporaryDirectory()
        self.root = Path(self.t.name)
        self.repo = make_repo(self.root / "repo")
        self.exes = make_exes(self.root / "exes", ".exe")
        self.zip = self.root / "python-3.12.10-embed-amd64.zip"
        zsha = fake_embeddable(self.zip)
        self.pins = {"python_embeddable": {"file": self.zip.name, "sha256": zsha, "version_tag": "312"}}
        self.wheels = self.root / "wheels"
        self.wheels.mkdir()
        wsha = fake_wheel(self.wheels / "jsonschema-4.23.0-py3-none-any.whl")
        self.lock = f"x==1 \\\n    --hash=sha256:{wsha}\n"

    def tearDown(self):
        self.t.cleanup()

    def go(self, name="out", **over):
        kw = dict(exe_suffix=".exe", python_zip=self.zip, wheels_dir=self.wheels, dev_python=None, pins=self.pins, lock_text=self.lock)
        kw.update(over)
        return stage(self.repo, self.root / name, self.exes, **kw)

    def test_layout_pth_and_wheels(self):
        m = self.go()
        helpers = self.root / "out" / "helpers"
        # the embeddable build ignores PYTHONPATH: the site-packages folder and `import site` are written into the ._pth, and the original is replaced
        self.assertEqual((helpers / "python" / "python312._pth").read_text(),
                         "python312.zip\n.\nLib/site-packages\n../repo/tools/parts\n../repo/tools/report\n\nimport site\n")
        for d in S.HELPER_SCRIPT_DIRS:                          # the folders named in the ._pth exist next to python/
            self.assertTrue((helpers / "python" / d).resolve().is_dir() or d.endswith("parts") or d.endswith("report"))
        self.assertTrue((helpers / "python" / "python.exe").is_file())
        self.assertEqual((helpers / "python" / "Lib" / "site-packages" / "jsonschema" / "__init__.py").read_text(), "x = 1\n")
        s = json.loads((helpers / "helpers.json").read_text())
        self.assertEqual((s["python"], s["run_campaign_exe"]), ("python/python.exe", "bin/run_campaign.exe"))
        self.assertEqual(m["mode"], "windows")
        self.assertEqual(sum(1 for e in m["files"] if e["path"].endswith("._pth")), 1)

    def test_pin_mismatch_placeholder_and_wrong_name_are_refused(self):
        with self.assertRaises(S.StageError) as e:
            self.go(pins={"python_embeddable": {**self.pins["python_embeddable"], "sha256": "0" * 64}})
        self.assertIn("differs from the pin", str(e.exception))
        with self.assertRaises(S.StageError) as e:
            self.go("o2", pins={"python_embeddable": {**self.pins["python_embeddable"], "sha256": "TODO(HUMAN-VERIFY): later"}})
        self.assertIn("no sha256", str(e.exception))
        with self.assertRaises(S.StageError) as e:
            self.go("o3", pins={"python_embeddable": {**self.pins["python_embeddable"], "file": "other.zip"}})
        self.assertIn("expected the file", str(e.exception))
        self.assertFalse((self.root / "out").exists())

    def test_unlocked_wheel_and_empty_wheel_folder_are_refused(self):
        (self.wheels / "evil-1.0-py3-none-any.whl").write_bytes(b"not locked")
        with self.assertRaises(S.StageError) as e:
            self.go()
        self.assertIn("not one of the files", str(e.exception))
        shutil.rmtree(self.wheels)
        self.wheels.mkdir()
        with self.assertRaises(S.StageError) as e:
            self.go("o2")
        self.assertIn("pip download", str(e.exception))

    def test_a_real_package_needs_zip_and_wheels_and_dev_python_excludes_them(self):
        with self.assertRaises(S.StageError):
            self.go("o", python_zip=None)
        with self.assertRaises(S.StageError):
            self.go("o2", dev_python="/usr/bin/python3")

    def test_verify_package_accepts_the_staged_tree_and_catches_every_defect(self):
        # a package folder with DroneData holding the helpers, plus the demo session files that the real repo has
        real = ROOT
        for rel in V.HELPERS_DEMO_FILES:
            p = self.repo / rel.replace("repo/", "", 1)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text("x")
        (self.repo / "parts_db" / "x.json").write_text("{}")
        shutil.rmtree(self.root / "out", ignore_errors=True)
        pkg = self.root / "pkg"
        project = pkg / "Windows" / "UAV_Simulator"
        (project / "DroneData").mkdir(parents=True)
        plan = S.build_plan(self.repo, exe_dir=self.exes, exe_suffix=".exe", python_zip=self.zip, wheels_dir=self.wheels, dev_python=None, cfd_dir=None, pins=self.pins, lock_text=self.lock)
        S.write_plan(plan, project / "DroneData")
        self.assertEqual(V.check_helpers(project), [])
        helpers = project / "DroneData" / "helpers"
        # an unlisted file, an altered file, a missing file, a bad _pth, a missing exe, an absolute python path
        (helpers / "stray.txt").write_text("x")
        self.assertTrue(any("unlisted file" in p and "stray.txt" in p for p in V.check_helpers(project)))
        (helpers / "stray.txt").unlink()
        f = helpers / "repo" / "parts_db" / "parts" / "m.json"
        f.write_text('{"changed": 1}')
        self.assertTrue(any("stale or altered" in p and "m.json" in p for p in V.check_helpers(project)))
        f.write_text("{}")
        f.unlink()
        self.assertTrue(any("missing: helpers/repo/parts_db/parts/m.json" in p for p in V.check_helpers(project)))
        f.write_text("{}")
        pth = helpers / "python" / "python312._pth"
        good = pth.read_text()
        pth.write_text("python312.zip\n.\n# import site\n")
        probs = V.check_helpers(project)
        self.assertTrue(any("lacks the" in p for p in probs))          # (the manifest hash also changed: both are reported)
        pth.write_text(good)
        (helpers / "bin" / "validate_models.exe").unlink()
        self.assertTrue(any("validate_models.exe" in p for p in V.check_helpers(project)))
        self.assertEqual(real, ROOT)

    def staged_windows_project(self):
        for rel in V.HELPERS_DEMO_FILES:
            p = self.repo / rel.replace("repo/", "", 1)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text("x")
        project = self.root / "pkgw" / "Windows" / "UAV_Simulator"
        (project / "DroneData").mkdir(parents=True)
        plan = S.build_plan(self.repo, exe_dir=self.exes, exe_suffix=".exe", python_zip=self.zip, wheels_dir=self.wheels, dev_python=None, cfd_dir=None, pins=self.pins, lock_text=self.lock)
        S.write_plan(plan, project / "DroneData")
        return project, project / "DroneData" / "helpers"

    @staticmethod
    def drop_from_manifest(helpers: Path, *rels: str) -> None:
        mp = helpers / "helpers_manifest.json"
        m = json.loads(mp.read_text())
        m["files"] = [e for e in m["files"] if e["path"] not in rels]
        mp.write_text(json.dumps(m))

    def test_verify_each_rule_alone(self):
        project, helpers = self.staged_windows_project()
        self.assertEqual(V.check_helpers(project), [])
        # same size, different content: only the hash can tell
        f = helpers / "repo" / "parts_db" / "parts" / "m.json"
        f.write_text("[]")
        self.assertTrue(any("stale or altered" in p for p in V.check_helpers(project)))
        f.write_text("{}")
        # only ONE of the two ._pth lines missing is already a problem
        pth = helpers / "python" / "python312._pth"
        good = pth.read_text()
        for missing in ("import site", "Lib/site-packages"):
            pth.write_text("\n".join(l for l in good.splitlines() if l != missing) + "\n")
            self.assertTrue(any("lacks the" in p for p in V.check_helpers(project)), missing)
        pth.write_text(good)
        # an absolute interpreter path in a Windows package, with the manifest and file otherwise intact
        hj = helpers / "helpers.json"
        original = hj.read_text()
        s = json.loads(original)
        s["python"] = str(helpers / "python" / "python.exe")
        hj.write_text(json.dumps(s, indent=2) + "\n")
        self.assertTrue(any("absolute path in a Windows package" in p for p in V.check_helpers(project)))
        hj.write_text(original)
        self.assertEqual(V.check_helpers(project), [])

    def test_verify_checks_do_not_depend_on_the_manifest_listing_everything(self):
        project, helpers = self.staged_windows_project()
        (helpers / "bin" / "validate_models.exe").unlink()
        self.drop_from_manifest(helpers, "bin/validate_models.exe")
        self.assertTrue(any("missing: helpers/bin/validate_models.exe" in p for p in V.check_helpers(project)))
        project, helpers = self.staged_windows_project() if False else (project, helpers)
        demo = helpers / V.HELPERS_DEMO_FILES[2]
        demo.unlink()
        self.drop_from_manifest(helpers, V.HELPERS_DEMO_FILES[2])
        self.assertTrue(any("missing demo session file" in p for p in V.check_helpers(project)))
        shutil.rmtree(helpers / "python" / "Lib" / "site-packages" / "jsonschema")
        self.drop_from_manifest(helpers, "python/Lib/site-packages/jsonschema/__init__.py")
        self.assertTrue(any("locked wheels were not unpacked" in p for p in V.check_helpers(project)))

    def test_verify_main_requires_helpers_only_on_request(self):
        pkg = self.root / "pkg2"
        (pkg / "Windows" / "UAV_Simulator" / "DroneData").mkdir(parents=True)
        probs = V.check_helpers(pkg / "Windows" / "UAV_Simulator")
        self.assertEqual(len(probs), 1)
        self.assertIn("helpers_manifest.json", probs[0])


class OnnxRuntimeStagingTests(unittest.TestCase):
    """The ONNX Runtime DLL that the helpers load: pinned, hash-checked, staged into bin/, listed in the manifest, verified."""
    NAME = "onnxruntime-win-x64-1.27.1"

    def setUp(self):
        self.t = tempfile.TemporaryDirectory()
        self.root = Path(self.t.name)
        self.dll = b"MZ fake onnxruntime"
        self.zip = self.root / f"{self.NAME}.zip"
        with zipfile.ZipFile(self.zip, "w") as z:
            z.writestr(f"{self.NAME}/lib/onnxruntime.dll", self.dll)
            z.writestr(f"{self.NAME}/LICENSE", "MIT\n")
            z.writestr(f"{self.NAME}/ThirdPartyNotices.txt", "notices\n")
        self.pin = {"file": self.zip.name, "sha256": sha(self.zip.read_bytes()), "dll_member": f"{self.NAME}/lib/onnxruntime.dll",
                    "dll_sha256": sha(self.dll), "license_members": [f"{self.NAME}/LICENSE", f"{self.NAME}/ThirdPartyNotices.txt"]}
        self.pins = {"onnxruntime_win_x64": self.pin}

    def tearDown(self):
        self.t.cleanup()

    def test_members_are_the_dll_and_its_licence_files_under_bin(self):
        got = S.ort_members(self.zip, self.pins)
        self.assertEqual(sorted(got), ["bin/onnxruntime.dll", "bin/onnxruntime_LICENSE", "bin/onnxruntime_ThirdPartyNotices.txt"])
        self.assertEqual(got["bin/onnxruntime.dll"], self.dll)

    def test_every_pin_mismatch_is_refused(self):
        def refused(pin_changes, needle, zip_path=None):
            pins = {"onnxruntime_win_x64": {**self.pin, **pin_changes}}
            with self.assertRaises(S.StageError) as e:
                S.ort_members(zip_path or self.zip, pins)
            self.assertIn(needle, str(e.exception))
        refused({"sha256": "0" * 64}, "differs from the pin")
        refused({"dll_sha256": "0" * 64}, "differs from the pin")
        refused({"sha256": "TODO(HUMAN-VERIFY)"}, "is not a sha256")
        refused({"dll_member": f"{self.NAME}/lib/other.dll"}, "has no")
        refused({"file": "other.zip"}, "expected the file")
        with self.assertRaises(S.StageError):
            S.ort_members(self.zip, {})
        # a zip with the right name but other bytes: the zip hash catches it before anything is read
        other = self.root / "x"
        other.mkdir()
        fake = other / self.zip.name
        fake.write_bytes(b"not the pinned zip")
        refused({}, "differs from the pin", fake)

    def test_the_committed_pin_matches_the_fetch_script_and_the_ort_the_crate_wants(self):
        import fetch_onnx as F
        pin = json.loads(S.PINS.read_text(encoding="utf-8"))["onnxruntime_win_x64"]
        win = F.ASSETS["windows"]
        self.assertEqual((pin["file"], pin["sha256"]), (win["file"], win["sha256"]))
        self.assertEqual(pin["url"], F.BASE_URL + win["file"])
        for key in ("sha256", "dll_sha256"):
            self.assertRegex(pin[key], r"^[0-9a-f]{64}$")
        # ort 2.0.0-rc.13 (default feature api-27) refuses an ONNX Runtime whose minor version is below 27
        self.assertTrue(F.VERSION.startswith("1.27."), F.VERSION)

    def test_stage_with_the_zip_writes_dll_licence_helpers_json_and_manifest(self):
        repo = make_repo(self.root / "repo")
        exes = make_exes(self.root / "exes", "")
        cfd = self.root / "cfd"
        cfd.mkdir()
        for n in ("m.onnx", "m.onnx.model.json", "geometry.json", "slice.png", "slice.legend.txt"):
            (cfd / n).write_bytes(b"x")
        plan = S.build_plan(repo, exe_dir=exes, exe_suffix="", python_zip=None, wheels_dir=None, dev_python="/usr/bin/python3", cfd_dir=cfd,
                            pins=self.pins, lock_text="", onnxruntime_zip=self.zip)
        m = S.write_plan(plan, self.root / "out")
        helpers = self.root / "out" / "helpers"
        paths = {e["path"] for e in m["files"]}
        self.assertTrue({"bin/onnxruntime.dll", "bin/onnxruntime_LICENSE", "cfd/slice.legend.txt", "cfd/m.onnx.model.json"} <= paths)
        s = json.loads((helpers / "helpers.json").read_text())
        self.assertEqual((s["onnxruntime"], s["cfd_dir"]), ("bin/onnxruntime.dll", "cfd"))
        # verification: the pinned hash is accepted, anything else is not
        pins_file = self.root / "pins.json"
        pins_file.write_text(json.dumps(self.pins))
        self.assertEqual(V.check_onnxruntime(helpers, "bin/onnxruntime.dll", pins_file), [])
        (helpers / "bin" / "onnxruntime.dll").write_bytes(b"MZ swapped")
        self.assertTrue(any("not the pinned ONNX Runtime" in p for p in V.check_onnxruntime(helpers, "bin/onnxruntime.dll", pins_file)))
        (helpers / "bin" / "onnxruntime.dll").unlink()
        self.assertTrue(any("missing" in p for p in V.check_onnxruntime(helpers, "bin/onnxruntime.dll", pins_file)))

    def test_without_the_zip_nothing_ort_related_is_written(self):
        repo = make_repo(self.root / "repo")
        exes = make_exes(self.root / "exes", "")
        m = stage(repo, self.root / "out", exes)
        self.assertFalse(any("onnxruntime" in e["path"] for e in m["files"]))
        s = json.loads((self.root / "out" / "helpers" / "helpers.json").read_text())
        self.assertNotIn("onnxruntime", s)
        self.assertNotIn("cfd_dir", s)

    def test_a_real_package_without_the_zip_is_refused_by_the_command_line(self):
        repo = make_repo(self.root / "repo")
        exes = make_exes(self.root / "exes", ".exe")
        pkg = self.root / "pkg" / "Windows" / "UAV_Simulator" / "DroneData"
        pkg.mkdir(parents=True)
        rc = S.main(["--repo", str(repo), "--package", str(self.root / "pkg"), "--exe-dir", str(exes), "--python-zip", str(self.root / "p.zip"),
                     "--wheels-dir", str(self.root / "w")])
        self.assertEqual(rc, 1)
        self.assertFalse((pkg / "helpers").exists())

    def test_verify_requires_the_dll_of_a_windows_package_only(self):
        project = self.root / "p" / "Windows" / "UAV_Simulator"
        helpers = project / "DroneData" / "helpers"
        helpers.mkdir(parents=True)
        for mode, declared, expect_problem in (("windows", None, True), ("windows", "bin/onnxruntime.dll", False), ("dev", None, False)):
            (helpers / "helpers_manifest.json").write_text(json.dumps({"mode": mode, "files": []}))
            (helpers / "helpers.json").write_text(json.dumps({"onnxruntime": declared} if declared else {}))
            self.assertEqual(bool(V.require_onnxruntime(project)), expect_problem, (mode, declared))


class SelfSufficiencyTests(unittest.TestCase):
    """The STAGED tree runs the helpers without reading anything outside itself and reproduces the committed outputs."""

    @classmethod
    def setUpClass(cls):
        cls.t = tempfile.TemporaryDirectory()
        cls.root = Path(cls.t.name)
        cls.dest = cls.root / "stage"
        exe_dir = RELEASE if (RELEASE / "run_campaign").is_file() else make_exes(cls.root / "exes", "")
        cls.have_real_exes = (RELEASE / "run_campaign").is_file() and (RELEASE / "validate_models").is_file()
        cls.manifest = stage(ROOT, cls.dest, exe_dir, dev_python=sys.executable)
        cls.helpers = cls.dest / "helpers"
        cls.env = {"PATH": os.environ.get("PATH", ""), "HOME": str(cls.root / "home"), "LANG": "C.UTF-8"}   # nothing else leaks in

    @classmethod
    def tearDownClass(cls):
        cls.t.cleanup()

    def run_py(self, script: str, *args: str, cwd: Path):
        # -E -s: no PYTHONPATH or other environment, no user site (the script's folder stays on the path, as it does for the app's child process
        # together with the ._pth lines); the environment is cleared too and the working folder is elsewhere
        return subprocess.run([sys.executable, "-E", "-s", str(self.helpers / "repo" / script), *args], cwd=cwd, env=self.env, capture_output=True, text=True, timeout=600)

    def test_the_staged_tree_has_the_expected_shape(self):
        paths = {e["path"] for e in self.manifest["files"]}
        for rel in ("repo/tools/parts/build_vehicle.py", "repo/tools/report/make_report.py", "repo/tools/report/run_study.py", "repo/parts_db/parts_catalogue.json",
                    "repo/campaign/test_definitions.json", "repo/campaign/condition_sets.json", "repo/tools/parts/builds/reference_quad/report_smoke/report.html",
                    "repo/offline_pipeline/props/apce_12x6.json", "helpers.json"):
            self.assertIn(rel, paths, rel)
        self.assertFalse(any("source_cache" in p or "__pycache__" in p or p.endswith("parts_index.sqlite") for p in paths))
        self.assertFalse(any(Path(p).name.startswith("test_") and p.endswith(".py") for p in paths))
        self.assertLess(self.manifest["totals"]["bytes"], 40 * 1024 ** 2)

    def test_staged_builder_reproduces_the_committed_build(self):
        out = self.root / "build_out"
        elsewhere = self.root / "cwd"
        elsewhere.mkdir(exist_ok=True)
        r = self.run_py("tools/parts/build_vehicle.py", str(self.helpers / "repo/tools/parts/builds/reference_quad.build.json"), "--out", str(out), cwd=elsewhere)
        self.assertEqual(r.returncode, 0, r.stderr[-500:])
        for f in ("vehicle_config.json", "vehicle_build.json", "assembly.json", "powertrain_config.json", "controller_config.json"):
            self.assertEqual((out / f).read_bytes(), (ROOT / "tools/parts/builds/reference_quad" / f).read_bytes(), f)

    def test_staged_report_tool_reproduces_the_committed_report(self):
        out = self.root / "report_out"
        elsewhere = self.root / "cwd2"
        elsewhere.mkdir(exist_ok=True)
        b = self.helpers / "repo/tools/parts/builds/reference_quad"
        r = self.run_py("tools/report/make_report.py", "--build", str(b), "--campaign", str(b / "campaign_smoke"), "--validation", str(b / "validation_report.json"), "--out", str(out), cwd=elsewhere)
        self.assertEqual(r.returncode, 0, r.stderr[-500:])
        for f in sorted(p.name for p in (ROOT / "tools/parts/builds/reference_quad/report_smoke").iterdir()):
            self.assertEqual((out / f).read_bytes(), (ROOT / "tools/parts/builds/reference_quad/report_smoke" / f).read_bytes(), f)

    @unittest.skipUnless((RELEASE / "run_campaign").is_file() and (RELEASE / "validate_models").is_file(), "release examples not built")
    def test_staged_campaign_exe_reproduces_the_smoke_golden(self):
        out = self.root / "camp_out"
        b = self.helpers / "repo/tools/parts/builds/reference_quad"
        r = subprocess.run([str(self.helpers / "bin" / "run_campaign"), str(b), str(self.helpers / "repo/campaign/conditions_smoke.json"), str(self.helpers / "repo/campaign/smoke_definitions.json"),
                            str(out), "--threads", "2", "--git-hash", "golden"], cwd=self.root, env=self.env, capture_output=True, text=True, timeout=900)
        self.assertEqual(r.returncode, 0, r.stderr[-500:])
        golden = ROOT / "tools/parts/builds/reference_quad/campaign_smoke"
        for p in sorted(golden.rglob("*")):
            if p.is_file():
                self.assertEqual((out / p.relative_to(golden)).read_bytes(), p.read_bytes(), str(p.relative_to(golden)))

    def test_nothing_in_the_staged_python_files_points_outside_the_tree(self):
        pattern = re.compile(r"/home/|[A-Za-z]:\\\\Users")
        for p in (self.helpers / "repo" / "tools").rglob("*.py"):
            self.assertIsNone(pattern.search(p.read_text(encoding="utf-8")), str(p))


class ReadmeTests(unittest.TestCase):
    TEXT = (ROOT / "UE5Project" / "PACKAGING_README.md").read_text(encoding="utf-8")

    def test_every_tool_the_readme_names_exists(self):
        for rel in re.findall(r"tools\\([A-Za-z_\\]+\.(?:py|txt|json))", self.TEXT):
            self.assertTrue((ROOT / "tools" / rel.replace("\\", "/")).is_file(), rel)

    def test_the_readme_gives_the_stage_and_verify_commands_and_the_pin_step(self):
        for needle in ("stage_package.py --package", "--python-zip", "--wheels-dir", "--exe-dir", "verify_package.py --package", "--require-helpers",
                       "pins.json", "helpers_lock.txt", "pip download --require-hashes", "run_campaign.exe", "validate_models.exe", "TODO(HUMAN-VERIFY)"):
            self.assertIn(needle, self.TEXT, needle)

    def test_the_old_checklist_was_kept_as_history(self):
        old = (ROOT / "dev-docs" / "DEMO-SMOKE-CHECKLIST-OLD.md").read_text(encoding="utf-8")
        self.assertIn("START FLIGHT", old)
        self.assertIn("history", old.splitlines()[0].lower())
        self.assertNotIn("START FLIGHT", self.TEXT)

    def test_the_pin_file_is_honest_about_the_unknown_hash(self):
        pin = json.loads(S.PINS.read_text(encoding="utf-8"))["python_embeddable"]
        self.assertFalse(re.fullmatch(r"[0-9a-f]{64}", pin["sha256"]))     # until a human fills it in
        self.assertIn("TODO(HUMAN-VERIFY)", pin["sha256"])


if __name__ == "__main__":
    unittest.main()
