# Tier: F
"""Packaging-readiness checks that need no Unreal: the staged-file list agrees between the manifest and
UAV_Simulator.Build.cs, every listed file exists, the verifier catches missing / stale files, the INI settings
the brief asks for are present, and no runtime source hard-codes an absolute path."""
import json
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import verify_package as vp  # noqa: E402

ROOT = vp.ROOT
SPEC = json.loads(vp.MANIFEST.read_text(encoding="utf-8"))
BUILD_CS = (ROOT / "UE5Project" / "Source" / "UAV_Simulator" / "UAV_Simulator.Build.cs").read_text(encoding="utf-8")
PLUGIN_BUILD_CS = (ROOT / "UE5Project" / "Plugins" / "DronePhysicsBridge" / "Source" / "DronePhysicsBridge" / "DronePhysicsBridge.Build.cs").read_text(encoding="utf-8")
CONFIG = ROOT / "UE5Project" / "Config"


def make_package(tmp: Path, skip: str | None = None, with_dll: bool = True) -> Path:
    proj = tmp / "Windows" / "UAV_Simulator"
    for rel in SPEC["data_files"]:
        if rel == skip:
            continue
        dest = proj / "DroneData" / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / rel, dest)
    if with_dll:
        dll = proj / "Binaries" / "Win64" / SPEC["dll"]
        dll.parent.mkdir(parents=True, exist_ok=True)
        dll.write_bytes(b"MZ fake dll")
    return tmp


class ManifestTests(unittest.TestCase):
    def test_every_listed_file_exists_in_the_repo(self):
        for rel in SPEC["data_files"]:
            self.assertTrue((ROOT / rel).is_file(), rel)

    def test_build_cs_list_equals_the_manifest(self):
        block = re.search(r"string\[\] DemoDataFiles = new string\[\] \{(.*?)\};", BUILD_CS, re.S)
        self.assertIsNotNone(block)
        listed = re.findall(r'"([^"]+)"', block.group(1))
        self.assertEqual(listed, SPEC["data_files"])

    def test_build_cs_stages_to_project_dir_and_optional_cfd_model(self):
        self.assertIn('"$(ProjectDir)/DroneData/" + Rel', BUILD_CS)
        for rel in SPEC["optional_data_files"]:
            self.assertIn(rel, BUILD_CS)

    def test_plugin_stages_the_dll_next_to_the_exe(self):
        self.assertIn('RuntimeDependencies.Add("$(TargetOutputDir)/physics_core.dll", Path.Combine(RustLibPath, "physics_core.dll"))', PLUGIN_BUILD_CS)

    def test_data_paths_resolver_looks_in_the_staged_folder_first(self):
        text = (ROOT / "UE5Project" / "Source" / "UAV_Simulator" / "DroneDataPaths.h").read_text(encoding="utf-8")
        self.assertLess(text.index("ProjectDir() / TEXT(\"DroneData\")"), text.index("ProjectContentDir() / TEXT(\"DroneData\")"))
        self.assertLess(text.index("ProjectContentDir() / TEXT(\"DroneData\")"), text.index("TEXT(\"..\")"))


class VerifierTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_complete_package_passes(self):
        pkg = make_package(self.root)
        self.assertEqual(vp.check(pkg), [])
        self.assertEqual(vp.main(["--package", str(pkg)]), 0)

    def test_missing_data_file_fails(self):
        skip = "UE5Project/Plugins/DronePhysicsBridge/Content/DroneEnvironments.json"
        pkg = make_package(self.root, skip=skip)
        problems = vp.check(pkg)
        self.assertEqual(problems, [f"missing: DroneData/{skip}"])
        self.assertEqual(vp.main(["--package", str(pkg)]), 1)

    def test_stale_data_file_fails(self):
        pkg = make_package(self.root)
        victim = next((pkg / "Windows" / "UAV_Simulator" / "DroneData").rglob("quad_ref_build_config.json"))
        victim.write_text("{}")
        problems = vp.check(pkg)
        self.assertEqual(len(problems), 1)
        self.assertIn("stale", problems[0])

    def test_missing_dll_fails(self):
        pkg = make_package(self.root, with_dll=False)
        problems = vp.check(pkg)
        self.assertEqual(len(problems), 1)
        self.assertIn("physics_core.dll", problems[0])

    def test_dll_that_differs_from_the_built_one_is_stale(self):
        pkg = make_package(self.root / "pkg")
        repo = self.root / "repo"
        built = repo / "physics_core" / "target" / "release" / SPEC["dll"]
        built.parent.mkdir(parents=True)
        built.write_bytes(b"MZ a newer dll")
        for rel in SPEC["data_files"]:  # the repo copy of every data file (same bytes as staged)
            dest = repo / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / rel, dest)
        problems = vp.check(pkg, repo=repo)
        self.assertEqual(len(problems), 1)
        self.assertIn("stale: Binaries/Win64/physics_core.dll", problems[0])
        # Same bytes: no problem.
        (pkg / "Windows" / "UAV_Simulator" / "Binaries" / "Win64" / SPEC["dll"]).write_bytes(b"MZ a newer dll")
        self.assertEqual(vp.check(pkg, repo=repo), [])

    def test_no_dronedata_folder_fails(self):
        (self.root / "Windows").mkdir()
        problems = vp.check(self.root)
        self.assertEqual(len(problems), 1)
        self.assertIn("not staged", problems[0])

    def test_optional_file_absent_is_not_a_problem(self):
        pkg = make_package(self.root)
        self.assertFalse(any("aero_quad_cfd" in p for p in vp.check(pkg)))


class ConfigTests(unittest.TestCase):
    def test_vsync_and_60_fps_cap(self):
        engine = (CONFIG / "DefaultEngine.ini").read_text(encoding="utf-8")
        self.assertRegex(engine, r"(?m)^t\.MaxFPS=60$")
        self.assertRegex(engine, r"(?m)^r\.VSync=1$")
        user = (CONFIG / "DefaultGameUserSettings.ini").read_text(encoding="utf-8")
        self.assertIn("bUseVSync=True", user)
        self.assertIn("FrameRateLimit=60", user)

    def test_windowed_fullscreen_1080p(self):
        user = (CONFIG / "DefaultGameUserSettings.ini").read_text(encoding="utf-8")
        self.assertIn("FullscreenMode=1", user)  # EWindowMode::WindowedFullscreen
        self.assertIn("ResolutionSizeX=1920", user)
        self.assertIn("ResolutionSizeY=1080", user)

    def test_scalability_is_low_or_medium(self):
        user = (CONFIG / "DefaultGameUserSettings.ini").read_text(encoding="utf-8")
        groups = re.findall(r"(?m)^sg\.(\w+)=(\S+)$", user)
        self.assertGreaterEqual(len(groups), 10)
        for name, value in groups:
            if name == "ResolutionQuality":
                continue
            self.assertIn(int(value), (0, 1), name)  # 0 low, 1 medium

    def test_game_mode_default_map_and_packaging_section(self):
        engine = (CONFIG / "DefaultEngine.ini").read_text(encoding="utf-8")
        self.assertIn("GlobalDefaultGameMode=/Script/UAV_Simulator.DroneDemoGameMode", engine)
        game = (CONFIG / "DefaultGame.ini").read_text(encoding="utf-8")
        self.assertIn("[/Script/UnrealEd.ProjectPackagingSettings]", game)
        self.assertIn("TODO(HUMAN-FILL)", game)

    def test_application_icon_placeholder_is_a_valid_ico(self):
        ico = (ROOT / "UE5Project" / "Build" / "Windows" / "Application.ico").read_bytes()
        self.assertEqual(ico[:4], b"\x00\x00\x01\x00")  # ICONDIR: reserved 0, type 1 (icon)
        self.assertEqual(int.from_bytes(ico[4:6], "little"), 1)  # one image

    def test_hud_class_is_set_by_the_game_mode(self):
        text = (ROOT / "UE5Project" / "Source" / "UAV_Simulator" / "DroneDemoGameMode.cpp").read_text(encoding="utf-8")
        self.assertIn("HUDClass = ADroneAppHUD::StaticClass();", text)   # P7: the application HUD replaced the demo HUD

    def test_slate_modules_are_declared(self):
        self.assertRegex(BUILD_CS, r'"Slate",\s*"SlateCore"')


class NoAbsolutePathTests(unittest.TestCase):
    def test_runtime_sources_have_no_absolute_paths(self):
        roots = [ROOT / "UE5Project" / "Source", ROOT / "UE5Project" / "Plugins" / "DronePhysicsBridge" / "Source"]
        pattern = re.compile(r'"[A-Za-z]:[\\/]|"/home/|"/Users/|"~/')
        offenders = []
        for base in roots:
            for path in list(base.rglob("*.cpp")) + list(base.rglob("*.h")):
                if "Generated" in path.parts and path.name == "drone_ffi.h":
                    continue
                for n, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
                    if pattern.search(line):
                        offenders.append(f"{path.relative_to(ROOT)}:{n}")
        self.assertEqual(offenders, [])


class ReadmeTests(unittest.TestCase):
    README = ROOT / "UE5Project" / "PACKAGING_README.md"

    def test_readme_exists_with_the_required_sections(self):
        text = self.README.read_text(encoding="utf-8")
        for needle in ("Package Project", "RunUAT", "BuildCookRun", "Standalone Game", "verify_package.py", "physics_core.dll", "Smoke test"):
            self.assertIn(needle, text, needle)

    def test_smoke_checklist_has_at_most_30_items(self):
        text = self.README.read_text(encoding="utf-8")
        section = text.split("## Smoke test", 1)[1]
        items = re.findall(r"(?m)^\d+\. ", section)
        self.assertTrue(10 <= len(items) <= 30, len(items))


if __name__ == "__main__":
    unittest.main()
