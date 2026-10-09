# Hazard matrix (DRAFT, UNREVIEWED)

Source: Phase 2 Completion Plan, Part 4.2. MIL-STD-882E style [UNVERIFIED].

Severity: I catastrophic, II critical, III marginal, IV negligible.
Likelihood: A frequent to E improbable.

These ratings are a first draft by the AI. The team must review and re-rate them. All references to
MIL-STD-882E, DO-178C and similar standards in our documents were AI written and need a human
check [UNVERIFIED].

Last column checked 2026-10-09 against `physics_core/TRACEABILITY.md`. "None" means no cargo test exists.
Item numbers refer to "Remaining work (A)" in `dev-docs/PHASE2-REALITY-CHECK.md`; B-n to its human-blocked table.

| ID | Hazard in our software | Sev. | Draft risk | Control (work package) | Tests today |
| :--- | :--- | :--- | :--- | :--- | :--- |
| HZ-01 | Stub or untrained model reported as validated | II | High | kind check, OSD warning (WP-0) | `surrogate::tests::test_is_stub_kind`, `tests::test_stub_loaded_flag_truth_table` (PASS, pure helpers). Loader refusal itself needs an ONNX file and `ORT_DYLIB_PATH` |
| HZ-02 | Missing structural model shows a safe margin | I | High | NaN and red flag (WP-7, WP-11) | `tests::test_ffi_step_no_models_reports_sources`, `types::tests::test_debug_telemetry_default_no_margin` (both PASS). Gap: the handle-present-but-query-failed path needs an ONNX session |
| HZ-03 | Result depends on frame rate | II | High | fixed step (WP-4) | `tests::test_ffi_invalid_dt` (dt rejection only). The 30/60/144 fps test is missing (B-6) |
| HZ-04 | Wrong or mirrored frame or sign | I | High | ADR-001, tests (WP-5) | 10 `mixer::tests::*` on the shipped config (hand-calculated yaw +-0.02 N*m, saturation, clamps, table) + `tests::test_ffi_step_positive_yaw_gives_positive_yaw_rate` (PASS). Spin per physical motor NOT confirmed against a real airframe (B-3); no engine photos (B-6) |
| HZ-05 | Placeholder mass or inertia in results | II | High | VehicleConfig (WP-2) | 15 `config::tests::*` plus 2 FFI code tests (all PASS). Validates finiteness and range only; placeholder values are still ASSUMED (docs/constants.md) |
| HZ-06 | Unit mistake in CFD, FEA or thermal data | II | Medium | unit module and hand checks (WP-3, 6, 7) | None (unit module not written, item 18) |
| HZ-07 | Motor temperature under predicted | III | Medium | thermal rewrite, bench test (WP-3, 9) | 6 `thermal::tests::*`: 3 PASS, 3 UNVERIFIED. No bench data |
| HZ-08 | Poisoned dependency or binary | I | Medium | pinned ONNX Runtime, SHA actions (WP-10) | No cargo test. Control is CI: `tools/fetch_onnx.py` SHA-256; action pinning is partial (10.7) |
| HZ-09 | AI wrote a wrong function that passed unnoticed | II | High | six gates, hand tests, traceability script (WP-0, 11) | No cargo test. Control is `tools/check_traceability.py` and its unit tests |
| HZ-10 | Model trained for a different geometry | II | Low | geometry hash gate (done), config hash (WP-2) | `surrogate::tests::test_check_provenance_*` (pure gate: geometry and config hash, missing keys, order), `tests::test_ffi_load_error_codes` (code 8). No test loads a real ONNX file (needs ORT, B-13) |

NaN/Inf divergence (WP-4.4): after the fault latch is set, all data produced until the caller clears it is
invalid. Nothing generated after the latch may be used as validation or pass/fail evidence. No HZ ID covers this
divergence yet; a human must decide whether to add one (draft, unreviewed). Test:
`tests::test_ffi_fault_latch_sticky_until_caller_clears`.
