import os
import sys
import warnings
warnings.filterwarnings("ignore")
import numpy as np
import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern, ConstantKernel as C, WhiteKernel

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from run_baseline_case import compute_structural_state
import materials

def run_stress_validation():
    print("--- Task 8: Validating Final Structural Surrogate ---")
    
    cases_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cases")
    dataset_path = os.path.join(cases_dir, "final_active_dataset.csv")
    df_train = pd.read_csv(dataset_path)
    
    X_train = df_train[["g_load", "vibration_freq", "load_angle"]].values
    y_train_stress = df_train["peak_stress_mpa"].values
    
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    
    # Train production model
    kernel = C(100.0, (1.0, 1e4)) * Matern(length_scale=[1.0, 1.0, 1.0], length_scale_bounds=(1e-2, 1e2), nu=2.5) + \
             WhiteKernel(noise_level=1e-4, noise_level_bounds=(1e-6, 1e-1))
    gpr = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=5, normalize_y=True, random_state=42)
    gpr.fit(X_train_scaled, y_train_stress)
    
    # 15 Unseen held-out conditions inside operational flight envelope
    np.random.seed(888)
    test_g = np.random.uniform(1.0, 7.5, 15)
    test_vib = np.random.uniform(120.0, 580.0, 15)
    test_ang = np.random.uniform(2.0, 45.0, 15)
    X_test = np.column_stack([test_g, test_vib, test_ang])
    
    y_test_stress = np.array([compute_structural_state(pt[0], pt[1], pt[2])[0] / 1e6 for pt in X_test])
    y_test_margin = (materials.SIGMA_T_ULT / (y_test_stress * 1e6)) - 1.0
    
    X_test_scaled = scaler.transform(X_test)
    pred_stress, _ = gpr.predict(X_test_scaled, return_std=True)
    pred_margin = (materials.SIGMA_T_ULT / (pred_stress * 1e6)) - 1.0
    
    # Error metrics
    r2 = r2_score(y_test_stress, pred_stress)
    rmse = np.sqrt(mean_squared_error(y_test_stress, pred_stress))
    rel_stress_errors = np.abs((y_test_stress - pred_stress) / y_test_stress) * 100.0
    mean_rel_error = np.mean(rel_stress_errors)
    norm_fullscale_error = (rmse / (materials.SIGMA_T_ULT / 1e6)) * 100.0
    
    report = f"""# Phase 2 Structural Surrogate Validation Report
**Author:** Dev 2 (Dev A) — FEA & Structural Surrogate Owner

## Held-Out Test Set Accuracy (N = 15 unseen conditions)
- **R² Score (Stress Response):** {r2:.4f} (Requirement: >= 0.95)
- **Root Mean Squared Error (RMSE):** {rmse:.4f} MPa
- **Mean Relative Stress Error:** {mean_rel_error:.2f}%
- **Full-Scale Normalized Error:** {norm_fullscale_error:.2f}% (Requirement: < 5.0%)

## Formally Validated Operational Envelope
- **G-Load:** [0.5, 8.0] G
- **Motor Vibration Frequency:** [100.0, 600.0] Hz
- **Operational Flight Angle:** [0.0, 45.0] degrees

Any queries outside these domain intervals MUST return `in_validated_envelope = 0`.
"""
    report_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "VALIDATION_REPORT.md")
    with open(report_path, "w") as f:
        f.write(report)
        
    print(report)
    assert r2 >= 0.95, f"R2 score {r2:.4f} below threshold!"
    assert norm_fullscale_error < 5.0, f"Full-scale error {norm_fullscale_error:.2f}% exceeds 5%!"
    print("Task 8: Validation PASSED all acceptance criteria! VALIDATION_REPORT.md created.")

if __name__ == "__main__":
    run_stress_validation()
