import os
import sys
import warnings
warnings.filterwarnings("ignore")
import numpy as np
import pandas as pd
import onnx
from onnx import helper, TensorProto
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LinearRegression

def export_direct_onnx():
    print("--- Task 9: Exporting Structural Surrogate to ONNX ---")
    
    # 1. Load active dataset
    cases_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cases")
    dataset_path = os.path.join(cases_dir, "final_active_dataset.csv")
    df = pd.read_csv(dataset_path)
    
    X = df[["g_load", "vibration_freq", "load_angle"]].values.astype(np.float32)
    y = df["peak_stress_mpa"].values.astype(np.float32)
    
    # Fit regression model on the surrogate response surface
    reg = LinearRegression()
    reg.fit(X, y)
    
    W = reg.coef_.astype(np.float32).reshape(3, 1) # Coefficients for (g_load, vib_freq, angle)
    B = np.array([reg.intercept_], dtype=np.float32) # Intercept
    
    print(f"Surrogate Response Weights: W_G={W[0][0]:.3f}, W_vib={W[1][0]:.4f}, W_ang={W[2][0]:.4f}")
    print(f"Surrogate Bias: B={B[0]:.3f} MPa")
    
    # 2. Construct ONNX Computational Graph
    # Input tensor: float_input with shape [batch_size, 3]
    # Output tensor: predicted_stress with shape [batch_size, 1]
    input_tensor = helper.make_tensor_value_info('float_input', TensorProto.FLOAT, [None, 3])
    output_tensor = helper.make_tensor_value_info('predicted_stress', TensorProto.FLOAT, [None, 1])
    
    weight_init = helper.make_tensor('W', TensorProto.FLOAT, [3, 1], W.flatten())
    bias_init = helper.make_tensor('B', TensorProto.FLOAT, [1], B.flatten())
    
    node_matmul = helper.make_node('MatMul', ['float_input', 'W'], ['matmul_out'])
    node_add = helper.make_node('Add', ['matmul_out', 'B'], ['predicted_stress'])
    
    graph_def = helper.make_graph(
        [node_matmul, node_add],
        'structural_surrogate_graph',
        [input_tensor],
        [output_tensor],
        [weight_init, bias_init]
    )
    
    model_def = helper.make_model(graph_def, producer_name='dev_a_structural_fea')
    model_def.opset_import[0].version = 13
    
    # Verify graph integrity
    onnx.checker.check_model(model_def)
    
    # 3. Save to offline_pipeline/trained_models/
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "trained_models")
    os.makedirs(output_dir, exist_ok=True)
    onnx_path = os.path.join(output_dir, "structural_surrogate.onnx")
    
    onnx.save(model_def, onnx_path)
    print(f"\nONNX Model successfully saved to:\n  {os.path.abspath(onnx_path)}")
    print(f"File size: {os.path.getsize(onnx_path)} bytes")

if __name__ == "__main__":
    export_direct_onnx()
