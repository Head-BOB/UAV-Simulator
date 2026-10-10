# Building and packaging the demo (Windows, RTX laptop)

Plain steps for the person with the Windows GPU laptop. Nothing here was run by the author (no Unreal on the author's machine):
if a step fails, copy the error text and send it back. Every number on screen is PRELIMINARY: most inputs are ASSUMED.

## 0. What you need
- Windows 10/11, Unreal Engine 5.8 (the project says `EngineAssociation 5.8`), Visual Studio with the "Game development with C++" workload.
- Rust through `rustup` (the repo pins the toolchain in `physics_core/rust-toolchain.toml`).
- Python 3.12 (only for the checks at the end).

## 1. Get the code and build the physics DLL
1. `git pull` on the `phase2` branch (or the branch you were told).
2. Open a terminal in `physics_core\` and run `cargo build --release`.
3. Check that `physics_core\target\release\physics_core.dll` and `physics_core.dll.lib` exist. (Unreal links the `.lib` and ships the `.dll`.)

## 2. Import the free environments (optional but wanted)
1. Download one free Fab environment per place: beach, rolling hills, mountains, desert, tropical.
2. Import each into the project, for example under `/Game/Environments/Beach/`.
3. Open `UE5Project\Plugins\DronePhysicsBridge\Content\DroneEnvironments.json` and write each level's asset path into `level_asset_path`
   (for example `/Game/Environments/Beach/L_Beach`). An empty path is fine: that environment then uses the default level and the screen says so.
4. Add each level to the cooked maps: in `UE5Project\Config\DefaultGame.ini`, under `[/Script/UnrealEd.ProjectPackagingSettings]`, add
   `+MapsToCook=(FilePath="/Game/Environments/Beach/L_Beach")` (one line per level).
5. Import the drone model as `/Game/Drone/Drone` and create the `/Game/Drone/M_MotionDisc` material as described in
   `UE5Project\Plugins\DronePhysicsBridge\README.md`. Then re-run `python tools\gen_demo_headers.py --check` (it must say nothing).

## 3. Try it in the editor first
1. Double-click `UE5Project\UAV_Simulator.uproject`. Let it build the plugin and the game module. If it fails to compile, send the errors.
2. Press Play. You should see the start menu. Pick an environment, press START FLIGHT, press T to take off.
3. Follow "What to check on the first run" in the plugin README (propeller axis, FR/FL/RR/RL, model yaw).

## 4. Package (File menu)
1. In the editor: File > Package Project > Windows. Choose an empty output folder.
2. Wait. The first cook takes a long time.
3. The program is `<output>\Windows\UAV_Simulator.exe`. Double-click it.

## 5. Package (command line, same result)
Replace the paths. `Development` keeps the console and the log lines the author needs (`DroneDemo-DIAG`); use `Shipping` only after everything works.
```
"C:\Program Files\Epic Games\UE_5.8\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun ^
  -project="C:\path\to\UAV-Simulator\UE5Project\UAV_Simulator.uproject" ^
  -noP4 -platform=Win64 -clientconfig=Development -build -cook -stage -pak -archive ^
  -archivedirectory="C:\path\to\Packaged" -utf8output
```

## 6. Check that the DLL and the data files came along
1. From the repo root run: `python tools\verify_package.py --package C:\path\to\Packaged`
2. It prints `package OK` or a list of `PROBLEM:` lines (missing or stale files). Fix the first one and package again.
3. By hand: `Packaged\Windows\UAV_Simulator\Binaries\Win64\physics_core.dll` must exist, and `Packaged\Windows\UAV_Simulator\DroneData\` must hold the JSON files
   (`quad_ref_build_config.json`, `DroneEnvironments.json`, `DroneModelMap.json`, the catalog, the models).
4. The reports are written to `Packaged\Windows\UAV_Simulator\Saved\Reports\<environment>\` (HTML, JSON, CSV).

## 7. If packaging fails
- Fallback: in the editor use the Play button's menu > Standalone Game (or "New Editor Window"). The same code and data run there.
- A missing `physics_core.dll` or `.lib`: go back to step 1.
- Missing data files: the file is not in the list in `UAV_Simulator.Build.cs`, or it does not exist in the repo. `python tools\verify_package.py` names it.
- "Rust FFI Version Mismatch" at startup: the DLL is older than the code (interface version 12). Rebuild it (step 1) and package again.

## Smoke test (30 items at most; tick each one)
1. The window opens windowed-fullscreen at 1080p, and the frame rate stays at the 60 fps cap (vsync on).
2. The start menu shows five environment cards, a title and the line "PRELIMINARY: most inputs are ASSUMED".
3. The model credit "Model: Animated Drone by hartwelkisaka, CC BY 4.0" is visible on the start menu.
4. A card without an installed level says "level not installed: default level".
5. Clicking a card highlights it with the cyan outline.
6. START FLIGHT hides the menu and shows the flight display (top bar, instruments, motor bars, right column).
7. Nothing covers the middle of the screen: the drone is visible.
8. The top bar shows mode, environment name and a clock.
9. T takes off; the mode changes to TAKEOFF then FLYING and the altitude tape follows.
10. The four propellers spin flat about their own shaft (no wobble); adjacent propellers turn opposite ways.
11. Above about 1700 RPM the propeller blades fade into the motion disc.
12. WASD or the arrow keys move the drone; the horizon, speed tape and compass follow.
13. The wind arrow in the compass points where the wind blows relative to the nose.
14. Press TAB: the control panel opens at the left, the mouse cursor appears, the drone hovers (pilot input paused).
15. Type a wind speed of 10 in the panel and press APPLY WEATHER: the wind arrow and the drone's drift change.
16. Type a wind speed of 20: the panel says "Rejected, nothing changed" and the wind stays as it was.
17. Type an altitude of 2500 and a temperature of 2: the air density on the right column drops (about 0.95 kg/m3) and the drone needs more throttle.
18. Type letters in a number field: the panel says "... is not a number".
19. Click another environment in the panel: the physics preset changes (density, wind).
20. TAB again closes the panel and gives control back.
21. The right column shows five live margins; none of them is red in a gentle hover.
22. Land with L: the mode shows LANDING then ON GROUND.
23. A moment after landing the report screen opens and says it is flying the standard mission (a few seconds).
24. The report shows a verdict, components ranked weakest first, causes, and suggestions marked TESTED or NOT TESTED.
25. The report lists all five environments with a verdict each, and the PRELIMINARY / ASSUMED banner.
26. EXPORT REPORT opens the folder with `report.html`, `report.json` and two CSV files; the HTML opens in a browser.
27. BACK TO FLIGHT returns to the flight display; R opens the report again.
28. Switch the drone model off (rename the mesh): the demo still runs with simple shapes and says so.
29. The Output Log shows `Drone Physics Rust Bridge successfully loaded and verified!` and no `Rust FFI Version Mismatch`.
30. Close the game, start it again, pick another environment: it starts without errors.
