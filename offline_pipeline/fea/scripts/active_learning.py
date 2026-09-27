import os
import sys
import numpy as np
import pandas as pd
from scipy.spatial.distance import cdist

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from train_surrogate import PARAM_BOUNDS, fit_gpr_model
from run_baseline_case import compute_structural_state

def run_active_learning(budget=40):
    """
    Task 7: Active-Learning Loop using Relevance = Uncertainty * Distance.
    """
    cases_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cases")
    bootstrap_path = os.path.join(cases_dir, "bootstrap_dataset.csv")
    df = pd.read_csv(bootstrap_path)
    
    X_train = df[["g_load", "vibration_freq", "load_angle"]].values
    y_train = df["safety_margin"].values
    
    print(f"--- Task 7: Starting Active Learning ---")
    print(f"Initial bootstrap dataset: {len(X_train)} points. Exploration budget: {budget} new cases.")
    
    # 1. Generate 1,000 candidate points across the flight envelope
    np.random.seed(101)
    cand_g = np.random.uniform(PARAM_BOUNDS["g_load"][0], PARAM_BOUNDS["g_load"][1], 1000)
    cand_vib = np.random.uniform(PARAM_BOUNDS["vibration_freq"][0], PARAM_BOUNDS["vibration_freq"][1], 1000)
    cand_ang = np.random.uniform(PARAM_BOUNDS["load_angle"][0], PARAM_BOUNDS["load_angle"][1], 1000)
    X_candidates = np.column_stack([cand_g, cand_vib, cand_ang])
    
    # Normalization scale factors so G-load and frequency have equal distance weight
    scales = np.array([
        PARAM_BOUNDS["g_load"][1] - PARAM_BOUNDS["g_load"][0],
        PARAM_BOUNDS["vibration_freq"][1] - PARAM_BOUNDS["vibration_freq"][0],
        PARAM_BOUNDS["load_angle"][1] - PARAM_BOUNDS["load_angle"][0]
    ])
    
    model = fit_gpr_model(X_train, y_train)
    
    # 2. Iteratively pick the most informative point
    for step in range(budget):
        # Predict uncertainty (sigma) across all remaining candidates
        _, sigmas = model.predict(X_candidates, return_std=True)
        
        # Calculate distance to nearest existing training point
        dist_matrix = cdist(X_candidates / scales, X_train / scales, metric='euclidean')
        min_dist = dist_matrix.min(axis=1)
        
        # Relevance = Uncertainty * Distance
        relevance = sigmas * min_dist
        best_candidate_idx = np.argmax(relevance)
        selected_point = X_candidates[best_candidate_idx]
        
        # Run physical solver on the highest-relevance point
        _, new_margin = compute_structural_state(selected_point[0], selected_point[1], selected_point[2])
        
        # Add to training set
        X_train = np.vstack([X_train, selected_point])
        y_train = np.append(y_train, new_margin)
        
        # Remove selected point from candidate pool
        X_candidates = np.delete(X_candidates, best_candidate_idx, axis=0)
        
        # Retrain GPR
        model = fit_gpr_model(X_train, y_train)
        
        if (step + 1) % 10 == 0:
            print(f"  Step {step + 1:2d}/{budget}: Dataset = {len(X_train)} | Peak candidate sigma = {sigmas.max():.4f}")
            
    # 3. Save full dataset
    final_df = pd.DataFrame(X_train, columns=["g_load", "vibration_freq", "load_angle"])
    final_df["safety_margin"] = y_train
    final_path = os.path.join(cases_dir, "final_active_dataset.csv")
    final_df.to_csv(final_path, index=False)
    print(f"\nActive learning complete! Final dataset ({len(final_df)} cases) saved to:")
    print("cases/final_active_dataset.csv")

if __name__ == "__main__":
    run_active_learning(budget=40)
