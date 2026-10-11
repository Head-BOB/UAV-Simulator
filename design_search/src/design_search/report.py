"""
Campaign Reporting & Honest Language Generator
Phase 3 - Dev 4 (Deliverable P8)
Strict Adherence to Part 8.4 Fixed Wording Standards
"""

import json
from typing import List, Dict, Any

FIXED_DISCLAIMER = "All performance numbers are PREDICTED by fast models. None is validated until a validation report exists."

def format_predicted_endurance(minutes: float, sigma_min: float = 1.5) -> str:
    """Format: 'Predicted hover endurance XX.X min (sigma X.X min)'"""
    return f"Predicted hover endurance {minutes:.1f} min (sigma {sigma_min:.1f} min)"

def format_predicted_structural_margin(rf: float, rf_conservative: float) -> str:
    """Format: 'Reserve factor X.X, conservative X.X (predicted)'"""
    return f"Reserve factor {rf:.1f}, conservative {rf_conservative:.1f} (predicted)"

def generate_campaign_summary(
    campaign_name: str,
    n_evaluations: int,
    finalists: List[Dict[str, Any]],
    output_path: str = "campaign_summary.json"
) -> Dict[str, Any]:
    """Generates the campaign audit report with honest terminology."""
    summary = {
        "disclaimer": FIXED_DISCLAIMER,
        "campaign_name": campaign_name,
        "evaluations_count": n_evaluations,
        "finalists_selected": len(finalists),
        "finalists": finalists
    }
    with open(output_path, "w") as f:
        json.dump(summary, f, indent=2)
    return summary
