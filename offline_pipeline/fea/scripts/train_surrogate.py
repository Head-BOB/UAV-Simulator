import os
import sys
import numpy as np
import pandas as pd
from scipy.stats import qmc
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C, WhiteKernel

# Import our validated physical solver
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from run_baseline_case import compute_structural_state

# Task 5: Defined Parameter Sweep Bounds
PARAM_BOUNDS = {
    "g_load": (0.5, 8.0),
    "vibration_freq": (100.0, 600.0),
    "load_angle": (0.0, 90.0)
}

def generate_bootstrap_dataset(n_samples=20, seed=42):
    """
    Task 6: Generate an initial space-filling Latin Hypercube Sampling (LHS) dataset.
    """
    print(f"Generating {n_samples} bootstrap points across the parameter sweep...")
    sampler = qmc.LatinHypercube(d=3, seed=seed)
    raw_sample = sampler.random(n=n_samples)
    
    # Scale to physical boundaries
    g_vals = qmc.scale(raw_sample[:, 0:1], PARAM_BOUNDS["g_load"][0], PARAM_BOUNDS["g_load"][1])
    vib_vals = qmc.scale(raw_sample[:, 1:2], PARAM_BOUNDS["vibration_freq"][0], PARAM_BOUNDS["vibration_freq"][1])
    ang_vals = qmc.scale(raw_sample[:, 2:3], PARAM_BOUNDS["load_angle"][0], PARAM_BOUNDS["load_angle"][1])
    
    records = []
    for g, vib, ang in zip(g_vals.flatten(), vib_vals.flatten(), ang_vals.flatten()):
        stress, margin = compute_structural_state(g, vib, ang)
        records.append({
            "g_load": round(float(g), 3),
            "vibration_freq": round(float(vib), 1),
            "load_angle": round(float(ang), 2),
            "peak_stress_pa": stress,
            "safety_margin": margin
        })
        
    df = pd.DataFrame(records)
    cases_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cases")
    os.makedirs(cases_dir, exist_ok=True)
    csv_path = os.path.join(cases_dir, "bootstrap_dataset.csv")
    df.to_csv(csv_path, index=False)
    print(f"Saved {n_samples} bootstrap cases to: cases/bootstrap_dataset.csv")
    return df

def fit_gpr_model(X, y):
    """
    Fits Gaussian Process Regression with anisotropic RBF kernel and noise modeling.
    """
    kernel = C(1.0, (1e-2, 1e3)) * RBF(length_scale=[2.0, 100.0, 20.0], length_scale_bounds=(1e-2, 1e4)) + \
             WhiteKernel(noise_level=1e-4, noise_level_bounds=(1e-6, 1e-1))
             
    gpr = GaussianProcessRegressor(
        kernel=kernel,
        n_restarts_optimizer=10,
        normalize_y=True,
        random_state=42
    )
    gpr.fit(X, y)
    return gpr

if __name__ == "__main__":
    df = generate_bootstrap_dataset(n_samples=20)
    X = df[["g_load", "vibration_freq", "load_angle"]].values
    y = df["safety_margin"].values
    
    print("Fitting initial GPR surrogate on bootstrap cases...")
    gpr = fit_gpr_model(X, y)
    
    # Test on a held-back query point (e.g. 4.5G, 320Hz vibration, 25 deg angle)
    test_point = np.array([[4.5, 320.0, 25.0]])
    pred_margin, pred_sigma = gpr.predict(test_point, return_std=True)
    _, true_margin = compute_structural_state(4.5, 320.0, 25.0)
    
    print("\nInitial Surrogate Test Query (4.5G, 320Hz, 25 deg):")
    print(f"  Surrogate Predicted Margin: {pred_margin[0]:.3f} +/- {pred_sigma[0]:.3f}")
    print(f"  True Solver Margin:         {true_margin:.3f}")
    print("Task 6 initial surrogate trained successfully!")
