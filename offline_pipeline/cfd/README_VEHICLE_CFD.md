# Vehicle CFD sweep: step by step (for the GPU laptop)

Results are **PRELIMINARY and UNVALIDATED**. FluidX3D is a separate program with a non-commercial licence (college project); nothing of it is
copied into this repo. Every FluidX3D API call in the generated `setup.cpp` is marked `TODO(HUMAN-VERIFY: matches FluidX3D 3.8)`: the first case
may need a small fix. The old quad-only tool (`README_QUAD_CFD.md`) still exists; this one works for any built vehicle (both classes today).

## What you need
Windows, NVIDIA RTX 4050 laptop (6 GB), FluidX3D 3.8 in `C:\tools\FluidX3D`, Visual Studio build tools, the repo with its `.venv` (Python 3.12).

## One-time setup
1. In `C:\tools\FluidX3D\src\defines.hpp` make these lines active: `#define FP16S`, `#define EQUILIBRIUM_BOUNDARIES`, `#define FORCE_FIELD`,
   `#define SUBGRID`. Comment out `#define GRAPHICS` (headless). `run_sweep.ps1` checks this and stops if not.
2. Run any case once and note the memory FluidX3D prints at startup. Work out **bytes per cell** for these defines (`TODO(HUMAN-VERIFY: B-7)`);
   the tools have no default for it on purpose.

## Order of work (sanity first)
Pick the build, for example `tools/parts/builds/reference_quad` (or `fpv_5in`). `BYTES` is your measured bytes per cell.

1. **Sphere check** (a case with a known answer):
   ```
   python offline_pipeline/cfd/sphere_validation.py generate-sphere --out offline_pipeline/cfd/generated/sphere --bytes-per-cell BYTES
   powershell -File offline_pipeline\cfd\generated\sphere\run_sweep.ps1 -FluidX3D C:\tools\FluidX3D
   ```
   Re = 1000, expected Cd about 0.438 (Schiller-Naumann correlation, `[UNVERIFIED]`).
2. **Generate the vehicle cases** (about a minute; nothing is written if a case does not fit the 300^3-cell or VRAM limits):
   ```
   python offline_pipeline/cfd/vehicle_sweep.py generate --build tools/parts/builds/reference_quad --out offline_pipeline/cfd/generated/ref_sweep --bytes-per-cell BYTES
   ```
   Speeds and angles come from `class_sweeps.json` (per vehicle class, ASSUMED). Use `--max-cases 1` first and time one case; the manifest lists
   `lattice_updates` per case so you can extrapolate. Propellers are left out of the geometry (a solid disc would act as a plate); `geometry.json` lists what was omitted.
3. **Grid-refinement pair** (one case at two resolutions):
   ```
   python offline_pipeline/cfd/sphere_validation.py generate-refinement --build tools/parts/builds/reference_quad --out offline_pipeline/cfd/generated/ref_refine --bytes-per-cell BYTES
   ```
   Run the `coarse` and `fine` folders like any other sweep.
4. **Run the sweep (overnight)**:
   ```
   powershell -File offline_pipeline\cfd\generated\ref_sweep\run_sweep.ps1 -FluidX3D C:\tools\FluidX3D
   ```
   Per case it prints the VRAM estimate and the GPU's free VRAM and REFUSES a case that does not fit. A refused or failed case is skipped and listed.
   A case whose drag along the flow is not positive is rejected by the collector (sign convention or setup wrong).
5. **Sanity report**:
   ```
   python offline_pipeline/cfd/sphere_validation.py report --sphere offline_pipeline/cfd/generated/sphere --refinement offline_pipeline/cfd/generated/ref_refine --out offline_pipeline/cfd/generated/cfd_sanity_report.json
   ```
   It reports the deviation of the sphere Cd and the change between the two grids. No tolerance is claimed. If you skip this step the model says `NOT_RUN`.
6. **Flow images**: `python offline_pipeline/cfd/flow_image.py --case-dir <case folder> --out <name>.png --scale 2` (needs `slice_y_mid.f32` in the case folder).

## Send back
`vehicle_cfd.csv` and `sweep_manifest.json` (and `geometry.json`) from the sweep folder, the `slice_y_mid.f32` files, `cfd_sanity_report.json`,
and the console text of the first case (any FluidX3D errors).

## Train the surrogate (anywhere)
```
python offline_pipeline/cfd/train_vehicle_aero.py train --build tools/parts/builds/reference_quad --csv vehicle_cfd.csv --out offline_pipeline/trained_models/aero_ref_cfd.onnx --sanity cfd_sanity_report.json
python offline_pipeline/cfd/train_vehicle_aero.py verify --build tools/parts/builds/reference_quad --model offline_pipeline/trained_models/aero_ref_cfd.onnx --csv vehicle_cfd.csv
```
- The model is `kind=trained` only if every row says "preliminary CFD, unvalidated" and there are 3 or more speeds. The synthetic sample gives `kind=stub`.
- It is bound to the build: `config_hash` (the build's `vehicle_config.json`), `geometry_hash` (the CFD geometry) and a `binding_hash`; `verify` fails if the build, the CSV
  or the model file changed.
- The model predicts FORCE only (body frame, N). Moments are in the CSV and report but not in the model.
- Try everything without a GPU: `offline_pipeline/cfd/sample/synthetic_<class>_cfd.csv` is SYNTHETIC (a formula, not CFD).

## Use it in a campaign (needs ONNX Runtime 1.27.x: `ORT_DYLIB_PATH`, or `onnxruntime.dll` next to the helper)
Put the model, its `.onnx.model.json` and the sweep's `geometry.json` in one folder (for example `offline_pipeline/cfd/generated/cfd_demo`) and:
```
cargo run --release --example run_campaign -- <build> <conditions> <definitions> <out> --aero-dir <that folder>
```
The helper picks the ONE model whose `config_hash` is the hash of this build's `vehicle_config.json`. The result files then carry
`cfd_surrogate:<kind>:<model sha256>` as the drag model and a `cfd_surrogate` note with the label shown everywhere:
- `kind=trained`: "CFD surrogate: PRELIMINARY, UNVALIDATED" (with the held-out error and the sanity status; `NOT_RUN` until step 5 above is done);
- the committed synthetic fixture (`kind=stub`, needs a helper built with `--features allow_stub_models`): "CFD surrogate: SYNTHETIC (formula, not CFD)";
- any other stub is refused. A model that matches the build but cannot be used stops the run with exit code 3 and `<out>/aero_model_status.json`;
  `--analytic-fallback` (explicit, never automatic) continues on the analytic drag and says so in every result file.
The explicit form `--aero-model <model.onnx> --aero-sidecar <model.onnx.model.json> --aero-geometry <geometry.json>` still works. Pass the CFD folder to
the report with `make_report.py --cfd-dir <folder>`: velocity-slice PNGs in it are embedded, or the report says "no flow images".

## Demo sweep for the 6 GB laptop (fewest cases, lower resolution)
`--profile demo` on `vehicle_sweep.py generate` and `sphere_validation.py generate-refinement`: 3 speeds x 5 attitudes = 15 cases (default 36) at 60 cells across
the span (default 100). The model has 5 coefficients per body axis; one speed's 5 attitudes determine them and 3 speeds give the held-out error. Time per case
is unknown until you measure one: `python offline_pipeline/cfd/sweep_time_plan.py --seconds-per-case <S> --measured-profile demo` prints the total for each
profile and the commands in order. All values are ASSUMED (`class_sweeps.json`).
