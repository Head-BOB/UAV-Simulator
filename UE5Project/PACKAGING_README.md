# Building, packaging and checking the UAV test bench (Windows, RTX laptop)

Plain, numbered steps for the person with the Windows GPU laptop. Nothing here was run by the author (no Unreal and no Windows on the author's
machine): if a step fails, copy the error text and send it back. Every number in the app is PRELIMINARY: most inputs are ASSUMED, and nothing has
been checked against hardware. The app replays RECORDED test runs; it is not live physics.

## 0. What you need
- Windows 10/11, Unreal Engine 5.8 (the project says `EngineAssociation 5.8`), Visual Studio with the "Game development with C++" workload.
- Rust through `rustup` (the repo pins the toolchain in `physics_core/rust-toolchain.toml`).
- Python 3.12 ONCE, only to download the packages for the bundled Python in step 3. The packaged app brings its own Python: the person who
  runs the finished app needs to install nothing.

## 1. Get the code and build the physics DLL and the two helper programs
1. `git pull` on the branch you were told.
2. Open a terminal in `physics_core\` and run `cargo build --release`.
3. In the same folder run `cargo build --release --features allow_stub_models --example run_campaign --example validate_models`.
   `allow_stub_models` is for the DEMO build only: it lets the helper load the ONE committed SYNTHETIC model (a formula, not CFD; recognised by its file
   hash, any other stub is still refused) so the app can show "CFD surrogate: SYNTHETIC (formula, not CFD)". A real, trained CFD model loads with or
   without the feature. A release for real use must be built WITHOUT it (`cargo build --release --example run_campaign --example validate_models`).
4. Check that these exist in `physics_core\target\release\`: `physics_core.dll`, `physics_core.dll.lib`, and in `examples\`: `run_campaign.exe`, `validate_models.exe`.
   (Unreal links the `.lib` and ships the `.dll`; the two `.exe` files are the helpers the app starts to run the tests.)

## 2. Import the free environments and the drone model (optional but wanted)
Follow `UE5Project\Plugins\DronePhysicsBridge\README.md` (environments, `/Game/Drone/Drone`, the `/Game/Drone/M_MotionDisc` material). Without them the app
still runs: the 3D view shows simple shapes. Then run `python tools\gen_demo_headers.py --check` (it must say nothing).

## 3. Prepare the bundled Python (once)
1. Download `python-3.12.10-embed-amd64.zip` from python.org (Windows embeddable package, 64-bit).
2. Compute its hash: `certutil -hashfile python-3.12.10-embed-amd64.zip SHA256`. Compare it with the checksum python.org shows for that release.
3. Open `tools\package\pins.json` and replace the `TODO(HUMAN-VERIFY)` text after `"sha256":` with that hash. The stager refuses a different file.
4. Download the packages the helpers need, exactly the hash-locked ones (no other file is accepted):
   ```
   python -m pip download --require-hashes --no-deps --only-binary=:all: --platform win_amd64 --python-version 3.12 --implementation cp --abi cp312 ^
     -r tools\package\helpers_lock.txt -d C:\path\wheels
   ```
   If `pip` says a hash is missing for a Windows file, stop and send the message: the lock may need regenerating (`tools\package\make_helpers_lock.py`).

## 3b. Download ONNX Runtime (once; the helpers need it to load a CFD model)
1. Download `onnxruntime-win-x64-1.27.1.zip` from
   `https://github.com/microsoft/onnxruntime/releases/download/v1.27.1/onnxruntime-win-x64-1.27.1.zip` (about 77 MB). It must be version 1.27.x:
   the `ort` crate of this project refuses an older one.
2. Compute its hash: `certutil -hashfile onnxruntime-win-x64-1.27.1.zip SHA256`. It must be
   `2e00414a63fdef0914cd5a5ede6c707844878e0c08e1b6693842f0451b2df2a1` (the value pinned in `tools\package\pins.json`; the stager refuses any other file).
   TODO(HUMAN-VERIFY): that value was taken from the GitHub release itself; compare it with a second source before you trust it (see HUMAN-TODO section 12).
3. Do NOT unzip it and do not put anything in `System32`. Keep the zip anywhere (for example `C:\path\onnxruntime-win-x64-1.27.1.zip`): step 6 takes
   `lib\onnxruntime.dll` out of it, checks its hash and writes it to `DroneData\helpers\bin\` next to `run_campaign.exe`, which is where the helpers look for it.
   (If a model cannot be loaded, the app says why in amber on the Conditions screen; it never falls back by itself.)

## 4. Try it in the editor first
1. Double-click `UE5Project\UAV_Simulator.uproject`. Let it build the plugin and the game module. If it fails to compile, send the errors
   (the UI code has never been compiled: expect a few).
2. In the editor the app finds the helpers through the folder above the project (needs `python` and `cargo` on the PATH), or through `helpers.json`.
3. Press Play. You should see the step bar `1 Vehicle ... 8 Report`.

## 5. Package (File menu)
1. In the editor: File > Package Project > Windows. Choose an empty output folder.
2. Wait. The first cook takes a long time.
3. The program is `<output>\Windows\UAV_Simulator.exe`. Do NOT start it yet: the helpers are added in step 6.

## 5b. Package (command line, same result)
Replace the paths. `Development` keeps the console and the log lines the author needs; use `Shipping` only after everything works.
```
"C:\Program Files\Epic Games\UE_5.8\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun ^
  -project="C:\path\to\UAV-Simulator\UE5Project\UAV_Simulator.uproject" ^
  -noP4 -platform=Win64 -clientconfig=Development -build -cook -stage -pak -archive ^
  -archivedirectory="C:\path\to\Packaged" -utf8output
```

## 6. Add the helper programs to the package
```
python tools\package\stage_package.py --package C:\path\to\Packaged --python-zip C:\path\python-3.12.10-embed-amd64.zip ^
  --wheels-dir C:\path\wheels --onnxruntime-zip C:\path\onnxruntime-win-x64-1.27.1.zip ^
  --exe-dir physics_core\target\release\examples --cfd-dir offline_pipeline\cfd\sample
```
It checks the Python zip against the pin and every package against the lock, checks the ONNX Runtime zip and its DLL against the pins, copies the helper
scripts, the parts database, the test definitions, the golden builds (a demo session you can open without running anything), and writes
`DroneData\helpers\helpers_manifest.json` with the hash of every file. `--cfd-dir` copies a CFD folder to `DroneData\helpers\cfd\` and writes it into
`helpers.json` (`cfd_dir`): the helper then uses the model in it whose `config_hash` is the hash of the built vehicle's `vehicle_config.json`.
`offline_pipeline\cfd\sample` is the SYNTHETIC demo folder (formula, not CFD; it matches only the reference quad's standard build); after a real GPU
sweep use that sweep's `cfd_demo` folder instead (see `offline_pipeline\cfd\README_VEHICLE_CFD.md`). `--onnxruntime-zip` is required for a real package.
The stager refuses to overwrite an earlier `helpers` folder: delete it first.

## 7. Check that everything came along (before the first launch)
1. From the repo root run: `python tools\verify_package.py --package C:\path\to\Packaged --require-helpers`
2. It prints `package OK` or a list of `PROBLEM:` lines (missing, stale, altered or unlisted files). Fix the first one and stage again.
3. By hand: `Packaged\Windows\UAV_Simulator\Binaries\Win64\physics_core.dll` must exist, and `...\DroneData\helpers\python\python.exe` too.
4. Results of the app are written to `...\UAV_Simulator\Saved\DroneApp\session\` (the build, the test runs, the report with `report.html`).

## 8. If something fails
- Fallback: in the editor use the Play button's menu > Standalone Game (or "New Editor Window"). The same code and data run there.
- A missing `physics_core.dll` or `.lib`: go back to step 1.
- The app says "Could not start ... Check helpers.json": the helpers were not staged (step 6) or `DroneData\helpers\helpers.json` points to a missing file.
  `verify_package.py --require-helpers` names the file.
- Missing data files: the file is not in the list in `UAV_Simulator.Build.cs`, or it does not exist in the repo. `python tools\verify_package.py` names it.
- "Rust FFI Version Mismatch" at startup: the DLL is older than the code (interface version 14). Rebuild it (step 1) and package again.
- The old demo's checklist is in `dev-docs\DEMO-SMOKE-CHECKLIST-OLD.md` (history only; that UI no longer exists).

## Smoke test (30 items at most; tick each one)
1. The window opens windowed-fullscreen at 1080p and the step bar `1 Vehicle ... 8 Report` is on top; steps 2 to 8 are dim.
2. Clicking a dim step shows why it is locked (for example "Choose a vehicle first").
3. Vehicle: Quadcopter and FPV quadcopter are listed; Fixed wing, Hexacopter and Underwater vehicle say "Not available yet" and cannot be chosen.
4. Choosing Quadcopter opens Parts (step 2 is underlined).
5. Parts: the category tabs start with frame, motor, esc, propeller, battery; typing in Search filters the list; a manufacturer filter works.
6. A part's detail shows mass and key values, each with a tag (Cited, Assumed, Calculated or Measured), the price (or n/a) and "Unverified".
7. "Add to build" puts the part in "Your build"; the parts total shows mass (before arms and cables).
8. An FPV frame cannot be added to the Quadcopter class (a message says so).
9. Choose a 400 mm quad frame, an MN3508 motor, a 12x6E propeller, a 4S battery and the T-Motor AIR 40A ESC; "Continue to Build" turns on.
10. Build: the name field takes letters, digits and underscore (a space is refused with a message); "Build vehicle" shows "Working ..." then design estimates.
11. The estimates show mass about 1.4 to 1.5 kg and thrust-to-weight about 6; the compatibility list shows Caution lines in amber (nothing red).
12. The 3D view shows the vehicle as plain boxes and cylinders, and the panel says "All stand-ins".
13. Going back to Parts and choosing the generic 20 A ESC, then Build: the builder refuses and "E-ESC-I" is shown in red; Conditions stays locked.
14. Choose the AIR 40A ESC again and build: it works again.
15. Conditions: all eight tests are ticked (Hover with payload, Wind and gusts, Climb, Endurance, Thermal soak, Maximum speed, FPV sprint, FPV punch-out).
16. Typing 1.5 as the battery state of charge and pressing "Run tests" is refused with a message that names the field and the range (nothing is changed silently).
17. Untick all tests except Climb. Change a limit (for example target_height_m); it shows "edited (was ...)" and a yellow "Edited" tag.
18. "Run tests" starts; the top bar shows the job; when it ends the status says tests finished and the report was written (the first run takes a while).
19. Run opens: seven instruments in the bottom bar, a top bar with the test name, and nothing covering the middle of the screen.
20. Space plays and pauses the replay; I opens the details panel (motors, power, temperatures, margins with the limit mark, two charts); Tab opens the controls.
21. The speed buttons change 1x to 64x; clicking the seek bar jumps; Restart goes back to the start; the text says "Recorded run, not live physics".
22. Results: the tests, breaking points (for example hover payload) with the criterion and step size, the limiting component and the validation summary. The CFD
    block depends on what the run used. With the synthetic sample folder: red "CFD surrogate: SYNTHETIC (formula, not CFD)", the held-out error, "NOT_RUN" and
    "no flow images". With a real model from a GPU sweep: amber "CFD surrogate: PRELIMINARY, UNVALIDATED", its held-out error, the sanity status and the image names
    with an "Open flow images" button. Without any CFD folder: "CFD: no CFD run". If a model could not be loaded the Conditions screen shows the reason in amber,
    nothing ran, and a "Run anyway on analytic drag" button; use it once and check that Results then says "CFD model failed to load: ...; results use analytic drag".
23. Results: no test is listed as "Not implemented"; an "Edited" banner is shown because a limit was edited. If you also ran Maximum speed, FPV sprint or FPV punch-out, a banner says they use the legacy drag and are not vehicle limits (try them once, they take longer).
24. Report: "Open report" opens `report.html` in the browser (print it to PDF from there); "Open folder" opens the session folder.
25. Compare: "Run upgrade study" starts a long job; afterwards a table shows deltas in green (better) and red (worse), rejected parts with the reason, and "not tested" rows.
26. Going back to Vehicle and choosing another class clears Build, Run, Results, Compare and Report (they become locked again).
27. Close the app, start it again: it opens on Vehicle without errors; the Output Log shows `Drone Physics Rust Bridge successfully loaded and verified!` and no `Rust FFI Version Mismatch`.
28. Delete `DroneData\helpers\python` and start the app, build a vehicle: the message says it could not start `python` and names helpers.json (then restore the folder).
