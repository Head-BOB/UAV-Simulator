# Structural Bench-Test Validation Plan
**Owner:** Dev A

To ensure the CalculiX FEA surrogate matches real-world carbon fiber deflection, the following physical bench tests are required once the first prototype frame is manufactured:

1. **Static Load Test:** Mount the central chassis to a rigid bench. Suspend calibrated weights (1kg, 2kg, 5kg) from the motor mounts to simulate thrust forces. Measure vertical deflection using a dial indicator.
2. **Frequency Vibration Test:** Mount the frame to a vibration table or run motors without propellers at sweeping RPMs. Use an accelerometer to identify the primary resonant frequencies of the arms.
3. **Validation Gate:** The measured physical deflection and resonant frequencies must match the `.onnx` surrogate predictions within a 5% margin of error. If they drift further, the material properties in the CFD/FEA solver must be recalibrated.