# Validation reference data

Inputs of `physics_core::validation` (the model-vs-published-data report). Nothing here is a model parameter.

| File | What | Provenance |
|---|---|---|
| `tmotor_mn3508_kv700_table.json` | 20 bench rows of the T-Motor MN3508 KV700 (11.1 V and 14.8 V, four voltage / propeller groups of 5 throttle rows, three distinct T-MOTOR carbon props, 50-100 % throttle) | CITED, extracted by `tools/parts/extract_tmotor_table.py` from the saved store page (sha256 pinned in `../source_pins.json`). Unverified: compare with the live page. |
| `acceptance_bands.json` | percent bands that flag a row | ASSUMED |
| `battery_curve_<battery id>.csv` (optional) | a real discharge curve; columns `soc_fraction,voltage_v,current_a,source`. Absent = the curve is reported `NO_PUBLISHED_DATA`. | You supply it; give the source in the `source` column. |

UIUC Ct/Cp are read from `offline_pipeline/props/*.json` (not copied here). Regenerate the table: `python tools/parts/extract_tmotor_table.py`.
