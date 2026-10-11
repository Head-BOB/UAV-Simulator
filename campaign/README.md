# Test campaign inputs

Inputs of the headless campaign engine (`physics_core/src/campaign.rs`, run with `physics_core/examples/run_campaign.rs`).

| File | What |
|---|---|
| `conditions_standard.json` | Standard-day conditions, full battery, Dryden turbulence selected (it only acts when a mean wind is set). |
| `conditions_smoke.json` | Same with the battery at 5 % so the endurance test reaches cutoff quickly. For CI and golden files only. |
| `test_definitions.json` | The five test definitions (hover + payload, wind + gust, climb, endurance, thermal soak). Every value is ASSUMED (`TODO(HUMAN-VERIFY)`); editing one needs a new `definitions_version`. |
| `smoke_definitions.json` | Short durations and coarse tolerances for CI. Not for engineering use. |

Run: `cd physics_core && cargo run --release --example run_campaign -- ../tools/parts/builds/reference_quad ../campaign/conditions_standard.json ../campaign/test_definitions.json <out dir> [--tests hover_payload,climb] [--threads 4] [--git-hash <hash>]`

Each test writes `manifest.json` (inputs, definition, part ids, versions, drag model, turbulence label, seeds), `result.json` (outcome, breaking point with criterion and resolution, metrics, notes), `events.json` (first / last time of every event, peaks) and `timeseries_*.csv`. `summary.csv` and `campaign_result.json` sit at the top. The files hold no wall-clock time: the same inputs give identical bytes, whatever the thread count.

Conditions: `ambient_c` is the air temperature at the site (thermal ambient and, with `site_altitude_m`, the density), `wind_from_deg` is the meteorological direction the wind comes FROM, `turbulence` is `none`, `dryden` (MIL-HDBK-1797 low-altitude form, UNVERIFIED, driven by the mean wind taken as the wind at 20 ft) or `simple` (fallback). `gust_level` only applies to `simple`.

Not in this phase: maximum speed and the two FPV tests (they wait for a better drag model than the legacy analytic one, which every manifest names).
