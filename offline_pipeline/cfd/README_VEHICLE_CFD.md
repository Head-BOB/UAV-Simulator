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

## Use it in a campaign (needs an ONNX Runtime that `ort` accepts)
```
cargo run --release --example run_campaign -- <build> <conditions> <definitions> <out> --aero-model <model.onnx> --aero-sidecar <model.onnx.model.json> --aero-geometry <geometry.json>
```
Every manifest then says `cfd_surrogate:<kind>:<model sha256>` and the result lists the steps by source and outside the validated envelope.
KNOWN PROBLEM: `ort 2.0.0-rc.13` rejects the ONNX Runtime 1.19.0 this repo pins (it asks for 1.27 or newer), so loading a model fails until that is settled (HUMAN-TODO).
