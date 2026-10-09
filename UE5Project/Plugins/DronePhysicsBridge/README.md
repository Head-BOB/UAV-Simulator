# DronePhysicsBridge and the Unreal demo (Windows)

DEMO, not flight code. The Rust core (`physics_core`) is the physics truth; Unreal draws it.
The demo is a quad that you fly from the keyboard or a gamepad, with steady wind and gusts.

**Status: NOT built or run by its author.** The Rust side and the UE-free C++ were tested on Linux.
The Unreal C++ (`DroneDemoPawn`, `DroneDemoGameMode`) has never been compiled. Expect to paste
compiler errors back if the first build fails.

Model: Animated Drone by hartwelkisaka, CC BY 4.0
(https://sketchfab.com/3d-models/animated-drone-d47d61f8a78d4cce9f985ad7ade3e403).

## Steps (Windows, Win64, Development Editor)

1. **Install.** Unreal Engine 5.8 (`UE5Project/UAV_Simulator.uproject` says `EngineAssociation 5.8`),
   Visual Studio 2022 with the "Game development with C++" workload, and Rust via rustup
   (`physics_core/rust-toolchain.toml` pins 1.96.0 and rustup fetches it).
2. **Build the Rust core.** In a terminal at the repo root:
   `cd physics_core` then `cargo build --release`.
   This writes `physics_core\target\release\physics_core.dll` and `physics_core.dll.lib`
   (the plugin's `DronePhysicsBridge.Build.cs` links the `.lib` from exactly that folder).
3. **Import the model** (skip if `/Game/Drone/Drone` already exists):
   `blender -b --python tools/model/prepare_drone.py -- "C:\DroneAssets\scene.gltf" "C:\DroneAssets\Drone.fbx"`,
   read the report (every REQUIRED bone must be present), then in the Content Browser create the
   folder `Drone`, import `Drone.fbx` as a **Skeletal Mesh** named `Drone`, with animations
   NOT imported. The asset must be at `/Game/Drone/Drone`.
4. **Motion-disc material (optional, editor).** Create `/Game/Drone/M_MotionDisc`: Blend Mode
   Translucent, Shading Model Unlit, a Scalar Parameter `Opacity` into Opacity and a Vector
   Parameter `Color` into Emissive Color. Without it the demo hides the motion discs and logs a Warning.
5. **Copy the DLLs** to `UE5Project\Binaries\Win64\` (create the folder if needed):
   `physics_core.dll` from step 2. `onnxruntime.dll` is NOT needed by this demo (it loads no
   surrogate model); if you want it there anyway, put it in the same folder.
   TODO(HUMAN-VERIFY): the final DLL staging for packaged builds is not decided (HUMAN-TODO B-13).
6. **Build.** Double-click `UE5Project\UAV_Simulator.uproject` and answer Yes to "rebuild modules",
   or build `UAV_SimulatorEditor` | `Development Editor` | `Win64` in Visual Studio, or run
   `"C:\Program Files\Epic Games\UE_5.8\Engine\Build\BatchFiles\Build.bat" UAV_SimulatorEditor Win64 Development "<repo>\UE5Project\UAV_Simulator.uproject"`.
7. **Play.** Nothing to set in the editor: `Config/DefaultEngine.ini` selects
   `/Script/UAV_Simulator.DroneDemoGameMode`, whose default pawn is `ADroneDemoPawn`. Open the
   default map and press Play (Alt+P). Press **T** to take off.

## Controls

| Keys | Action |
| :--- | :--- |
| T / L | take off (to 2 m) / land |
| W S / A D | forward, back / left, right (heading frame, up to 3 m/s) |
| Q E | yaw left / right (1 rad/s) |
| Space / Left Ctrl | climb / descend (1.5 m/s) |
| Up / Down arrows | steady wind strength +1 / -1 m/s (0 to 15) |
| Left / Right arrows | wind direction -15 / +15 degrees (direction the wind blows TOWARD, from north) |
| G | gusts on / off |
| Backspace | zero wind (steady 0, gusts off) |
| Gamepad | left stick move, right stick yaw and climb, A takeoff, B land |

The HUD shows airspeed (relative to the air), altitude, wind (total, steady setting, gusts), and
RPM per motor, plus the model credit. No thermal values are shown. Cyan arrow: total wind. Orange
arrow: the gust part.

## What to check on the first run

1. The Output Log has no `Rust FFI Version Mismatch` (interface version 11) and no `static_assert` failure at build.
2. One `DroneDemo: uniform mesh scale ...` line: scale about 2.342 and a front-rear leftover of
   about -0.0597 m (the model is not square, the config is; props are not distorted).
3. No `DroneDemo: bone '...' is missing` Error. If you see one, the demo draws simple shapes instead.
4. Watch the FR prop at low throttle: it must turn counter-clockwise seen from above. If it
   spins about the wrong axis or the wrong way, change `kSpinAxisBoneLocal` or
   `kSpinDirectionScale` in `Public/DroneVisualMath.h` (the only place).
5. Confirm FR/FL/RR/RL against `DroneModelMap.json` (its own `TODO(HUMAN-VERIFY)`), the model's
   forward direction (`kMeshYawOffsetDeg` in `DroneDemoPawn.cpp`), and the ground height.

## Notes

- No surrogate models are loaded, so the Rust step uses the analytic drag model, which the wind acts on.
- The Rust core has no ground contact. The demo pins the vehicle level and at rest on the ground
  (`DroneDemoControl.h`, `ShouldTouchDown`), keeping its heading.
- Every demo number (speeds, gains, wind defaults, 15 m/s clamp, seed 12345) is
  `TODO(HUMAN-VERIFY: demo value)`; they are not physics constants.
- Unreal-free tests (from the repo root, Linux or Windows with g++): see `tools/cpp_ffi_check/`.
