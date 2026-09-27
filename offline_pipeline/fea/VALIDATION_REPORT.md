# Phase 2 Structural Surrogate Validation Report
**Author:** Dev 2 (Dev A) — FEA & Structural Surrogate Owner

## Held-Out Test Set Accuracy (N = 15 unseen conditions)
- **R² Score (Stress Response):** 0.9928 (Requirement: >= 0.95)
- **Root Mean Squared Error (RMSE):** 14.8061 MPa
- **Mean Relative Stress Error:** 9.35%
- **Full-Scale Normalized Error:** 0.99% (Requirement: < 5.0%)

## Formally Validated Operational Envelope
- **G-Load:** [0.5, 8.0] G
- **Motor Vibration Frequency:** [100.0, 600.0] Hz
- **Operational Flight Angle:** [0.0, 45.0] degrees

Any queries outside these domain intervals MUST return `in_validated_envelope = 0`.
