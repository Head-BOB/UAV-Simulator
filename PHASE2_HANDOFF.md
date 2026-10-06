# Phase 2 Interface Handoff

## To All Developers (Dev A, B, C)
Phase 2 FFI architecture is complete and locked.

1. **Precision Upgrade:** `DroneState.position` is now `f64`. It uses a strictly local North-East-Down (NED) coordinate frame relative to a WGS84 origin. Do not pass UE5 world coordinates into the Rust core.
2. **Surrogate AI Pipeline:** Do not write ML inference logic by hand. Use `surrogate::query_model()` which returns a `SurrogateQueryResult`.
3. **Geometry Provenance:** Every `.onnx` model exported from the `offline_pipeline` must contain the `geometry_hash` metadata, or the engine will refuse to load it (`GeometryMismatch` error code 3).
4. **Telemetry:** `DebugTelemetry` now includes motor temperatures, structural safety margin, and the `is_validated_envelope` flag. The OSD will glow orange if you fly outside your model's training data.