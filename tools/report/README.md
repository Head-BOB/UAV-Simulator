# Engineer's report and upgrade study (P6)

Reads campaign result folders written by `physics_core/examples/run_campaign.rs` and the build's files; writes one report.
Nothing is estimated here: every number in the report is read from a file written by another tool, and the report lists every
ASSUMED value that influenced it. Python standard library only (plus the parts-database tools it imports).

## One campaign, one report

```
python tools/report/make_report.py --build tools/parts/builds/reference_quad \
    --campaign tools/parts/builds/reference_quad/campaign_smoke \
    --validation tools/parts/builds/reference_quad/validation_report.json --out <report dir>
```
Optional: `--cfd-dir <dir>` (geometry.json, sweep_manifest.json, cfd_sanity_report.json, `*.model.json`, `*.png` of the CFD pipeline),
and several `--campaign` folders (one per condition set; the first one drives the detail sections, all appear in the envelope).

Outputs: `report.html` (self-contained, has a print stylesheet; use the browser's "print to PDF"), `report.json` (all numbers),
`summary.csv`, `margins.csv`, `events.csv`, `upgrades.csv`, `assumptions.csv`, `validation.csv`. Same inputs give identical bytes.

## Upgrade study

```
python tools/report/run_study.py tools/parts/builds/reference_quad.build.json \
    --conditions campaign/conditions_standard.json --definitions campaign/test_definitions.json --out <study dir> --threads 4
python tools/report/make_report.py --study <study dir> --out <report dir>
```
* Every part of the four roles battery / ESC / motor / propeller in the database is tried as a SINGLE swap. The builder's
  compatibility checker decides: refused swaps are listed with their rule id, the others run the same campaign under the same
  conditions as the baseline. A failed campaign is "not tested", never estimated.
* Ranking = change of the BASELINE's limiting margin; the other changes (payload, wind, endurance, temperatures, mass, price) are shown next to it,
  green better and red worse. A candidate with a scaled stand-in propeller table is shown but not ranked.
* The condition set is the one in `--conditions`; its `name` is written to every manifest. `--all-condition-sets` (default off) also runs the
  baseline on every set in `campaign/condition_sets.json`; `--candidates-on all` runs the candidates on all of them too.
* `--reuse` skips a campaign whose result folder exists. The study stops before running anything if it would need more than
  `--max-campaigns` (default 40) campaigns.
* The campaigns are run by `cargo run --release --example run_campaign`. A full-length campaign takes about half a minute per build on a laptop.

## Reading the report
* **Margin** = limit / peak load from the recorded runs. Below 1.0 the limit was exceeded. Temperatures use the rise above ambient.
  The limits come from the part data in `powertrain_config.json`; several are ASSUMED (see the Assumptions section).
* **Smoke** definitions (`campaign/smoke_definitions.json`) are short and start with an almost empty battery: the report says
  "NOT FOR ENGINEERING USE". Use `campaign/test_definitions.json` for real numbers.
* Not built yet and shown as "not tested": maximum speed and the FPV sprint tests. No CFD run exists unless `--cfd-dir` points at one.
