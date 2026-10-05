import os
import json
import hashlib
from datetime import datetime
import numpy as np
import onnx
from onnx import helper, TensorProto

def generate_fea_surrogate():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    geo_path = os.path.join(base_dir, 'geometry', 'current_geometry.json')

    with open(geo_path, 'rb') as f:
        geo_data = f.read()
    geo_hash = hashlib.sha256(geo_data).hexdigest()

    with open(geo_path, 'r') as f:
        geo_json = json.load(f)
    design_rev = str(geo_json.get("design_revision", 1))

    X_info = helper.make_tensor_value_info('inputs', TensorProto.FLOAT, [1, 1])
    Y_pred_info = helper.make_tensor_value_info('predicted_values', TensorProto.FLOAT, [1, 1])
    Y_unc_info = helper.make_tensor_value_info('uncertainty', TensorProto.FLOAT, [1, 1])
    Y_env_info = helper.make_tensor_value_info('in_validated_envelope', TensorProto.FLOAT, [1, 1])

    w_data = np.array([[-0.005]], dtype=np.float32)
    b_data = np.array([[15.0]], dtype=np.float32)

    node_w = helper.make_node(
        'Constant',
        inputs=[],
        outputs=['W'],
        value=helper.make_tensor(name='W_val', data_type=TensorProto.FLOAT, dims=[1, 1], vals=w_data.flatten().tolist())
    )

    node_b = helper.make_node(
        'Constant',
        inputs=[],
        outputs=['B'],
        value=helper.make_tensor(name='B_val', data_type=TensorProto.FLOAT, dims=[1, 1], vals=b_data.flatten().tolist())
    )

    node_matmul = helper.make_node('MatMul', inputs=['inputs', 'W'], outputs=['mx'])
    node_pred = helper.make_node('Add', inputs=['mx', 'B'], outputs=['predicted_values'])

    node_unc = helper.make_node(
        'Constant',
        inputs=[],
        outputs=['uncertainty'],
        value=helper.make_tensor(name='unc_val', data_type=TensorProto.FLOAT, dims=[1, 1], vals=[0.02])
    )

    node_env = helper.make_node(
        'Constant',
        inputs=[],
        outputs=['in_validated_envelope'],
        value=helper.make_tensor(name='env_val', data_type=TensorProto.FLOAT, dims=[1, 1], vals=[1.0])
    )

    graph_def = helper.make_graph(
        [node_w, node_b, node_matmul, node_pred, node_unc, node_env],
        'FEASurrogate',
        [X_info],
        [Y_pred_info, Y_unc_info, Y_env_info]
    )

    model_def = helper.make_model(graph_def, producer_name='uav-simulator-fea')

    model_def.metadata_props.add(key="geometry_hash", value=geo_hash)
    model_def.metadata_props.add(key="design_revision", value=design_rev)
    model_def.metadata_props.add(key="trained_at_utc", value=datetime.utcnow().isoformat())
    model_def.metadata_props.add(key="trained_by", value=os.environ.get("GITHUB_SHA", "local_dev"))

    out_path = os.path.join(base_dir, 'trained_models', 'fea_surrogate.onnx')
    onnx.save(model_def, out_path)

if __name__ == '__main__':
    generate_fea_surrogate()