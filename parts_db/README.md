# parts_db: parts database with provenance

Source of truth: `parts/<category>/<id>.json` (one record per file, sorted keys, LF). Schema: `schema/part.schema.json`
(generated from `tools/parts/fields.py`; do not edit by hand). Index: `parts_index.sqlite` (generated, gitignored).
Design and review notes: `dev-docs/PARTS-DB.md`. Checklist for the human: `dev-docs/PARTS-VERIFY.md`.

Every number is `{"value": ..., "provenance": {"source_type": MEASURED|CITED|CALCULATED|ASSUMED, ...}}`.
A record with a bare number, or a CITED value without URL/date, or an ASSUMED value without `TODO(HUMAN-VERIFY)`, is refused.

## Commands (repo root, inside `.venv`)
    python tools/parts/parts_cli.py validate
    python tools/parts/parts_cli.py search --category motor --max-mass 0.12 --text t-motor
    python tools/parts/parts_cli.py search --category battery --compatible-with motor=tmotor_mn3508_kv700,esc=tmotor_air_20a_3_4s
    python tools/parts/parts_cli.py show tmotor_mn3508_kv700
    python tools/parts/parts_cli.py add --from my_part.json            # refuses incomplete records
    python tools/parts/import_parts.py --category motor my_motors.csv  # CSV route, all-or-nothing
    python tools/parts/parts_cli.py tick tmotor_mn3508_kv700 kv_rpm_per_v --by "Your Name"   # after checking the datasheet
    python tools/parts/fetch_source.py URL                              # pin one manufacturer page (allowlist only)

## Honest limits
- No automatic scraping and no "paste a link and extract" feature. Values are typed by a person or an AI reader from a
  pinned page and stay **Unverified** until a human ticks them.
- The pinned hash identifies the saved copy of a page, not the live page (pages change on every fetch).
- Prop Ct/Cp tables exist only for the APC 12x6E (UIUC). Other propellers are SCALED from it and marked ASSUMED.
