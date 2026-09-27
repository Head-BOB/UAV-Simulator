# Phase 2 FEA Solver Evaluation & Decision (Dev 2 / Dev A)

## Selected Solver: CalculiX (ccx)

### Justification:
1. **Headless Automation:** CalculiX runs headlessly from the command line (ccx -i jobname), making it easy to automate from Python scripts during the active-learning loop.
2. **Standard Input Format:** Uses the Abaqus .inp deck format, allowing programmatic definition of nodes, composite properties, and load boundary conditions.
3. **Static & Dynamic Analysis:** Native support for 3D shell and solid stress calculations, ideal for evaluating arm root bending and motor vibration harmonics.
