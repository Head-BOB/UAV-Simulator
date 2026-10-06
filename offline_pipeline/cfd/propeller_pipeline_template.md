# Custom Propeller CFD Pipeline Template
**Owner:** Dev B

For any new custom propeller geometry, follow this pipeline to generate its thrust/torque curve:

1. **UIUC Validation:** Before testing the custom prop, run the CFD solver (FluidX3D) on the standard `UIUC_Master_10x4.5` geometry. Confirm the predicted $C_T$ (Thrust Coefficient) matches the public wind-tunnel data.
2. **Rotating Geometry:** Import the new `.step` file of the custom propeller. Set the simulation to re-voxelize the bounding box every timestep to capture the rotation.
3. **Sweep:** Run sweeps at 1000, 3000, 5000, and 8000 RPM.
4. **Export:** Update `current_geometry.json` with the new design revision, run `train_aero.py` to bake the hash, and push the new `aero_surrogate.onnx` model to the live engine.