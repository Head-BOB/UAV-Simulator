# Quad CFD sweep: step by step (for the GPU laptop)

Results are PRELIMINARY and UNVALIDATED (no wind-tunnel data for this vehicle). FluidX3D is a separate program with a
non-commercial licence (college project); nothing of it is copied into this repo. Every FluidX3D API call in the
generated `setup.cpp` is marked `TODO(HUMAN-VERIFY: matches FluidX3D 3.8)`: the first case may need a small fix.

## What you need
- Windows, NVIDIA RTX 4050 laptop (6 GB), FluidX3D 3.8 in `C:\tools\FluidX3D`, Visual Studio build tools, Python 3.12 venv of this repo.

## One-time setup
1. In `C:\tools\FluidX3D\src\defines.hpp` make these lines active (not commented): `#define FP16S`, `#define EQUILIBRIUM_BOUNDARIES`,
   `#define FORCE_FIELD`, `#define SUBGRID`. Comment out `#define GRAPHICS` (headless). `run_sweep.ps1` checks this and stops if not.
2. Measure how many bytes per cell FluidX3D uses with those defines (it prints the memory use at startup for any case). Write the number down: `BYTES`.
   It is not guessed here (`TODO(HUMAN-VERIFY: B-7)`).

## Generate the cases (about 1 minute)
```
python offline_pipeline/cfd/fluidx3d_quad_sweep.py generate --bytes-per-cell BYTES
```
- Writes `offline_pipeline/cfd/generated/quad_sweep/` with 36 case folders (speeds 5, 10, 15, 20 m/s; angle of attack -20..20 step 10 at zero sideslip;
  sideslip -20..20 step 10 at zero angle of attack), `sweep_manifest.json` and `run_sweep.ps1`.
- Every case is 450 x 226 x 226 = 22,984,200 cells (limit 300^3 = 27,000,000). If `BYTES` is too large for 6 GB the command refuses and writes nothing.
- To run fewer cases: `--max-cases 8`, or `--speeds 10 20`, or `--angles -10 0 10`.
- 0 m/s is not simulated: with no relative flow the aero force is exactly zero.

## Run (overnight)
```
powershell -File offline_pipeline\cfd\generated\quad_sweep\run_sweep.ps1 -FluidX3D C:\tools\FluidX3D
```
- Per case it prints the VRAM estimate and the GPU's free VRAM, and REFUSES the case if it does not fit. A failed or refused case is skipped and listed.
- If the build command or exe path differs, pass `-BuildCmd "..."` and `-ExeRel "..."` (defaults are guesses: `TODO(HUMAN-VERIFY)`).
- Time per case: unknown until you run one (it depends on the GPU speed); each case is 30,000 lattice steps. Run ONE case first (`--max-cases 1`) and time it.
- A case where the drag along the flow is not positive is rejected by the collector (it means the force sign or the setup is wrong).

## Send back
`quad_cfd.csv` and `sweep_manifest.json` from the `quad_sweep` folder, plus the console text of the first case (any FluidX3D errors).

## Train the surrogate (anywhere)
```
python offline_pipeline/cfd/train_aero_from_csv.py --csv path\to\quad_cfd.csv --out offline_pipeline/trained_models/aero_quad_cfd.onnx
```
- `kind` is `trained` only if every row says "preliminary CFD, unvalidated"; the model carries the held-out error (leave-one-speed-out, N) and its valid range.
- Try the tools without a GPU: `--csv offline_pipeline/cfd/sample/synthetic_quad_cfd.csv` (SYNTHETIC numbers from a formula; gives `kind=stub`).
- The Rust core only loads a model whose `config_hash` equals the one compiled in from `vehicle_config.json`. This model records the hash of
  `quad_ref_build_config.json`, so the core will refuse it until the build hash is changed (decision for you, see HUMAN-TODO). Until then the demo flies on analytic drag and says so.
