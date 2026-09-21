import numpy as np
import onnx
import onnxruntime as ort
from onnx import numpy_helper


# Quick inference test using a saved ONNX model

# Load ONNX Runtime inference session
# Strictly, this is the only thing needed for inference
onnx_path = "/src/astrobee-isd/data/training/il_first_data/bc_policy/policy.onnx"
ort_sess = ort.InferenceSession(onnx_path)

# Load model, if you want to extract saved metadata
m = onnx.load(onnx_path)
init = {i.name: numpy_helper.to_array(i) for i in m.graph.initializer}
mean, sigma = init["mean"], init["sigma"]

# Create some random data, based off mean/sigma
rng = np.random.default_rng(0)
obs_arr = (mean + sigma * rng.standard_normal((100, 18))).astype(np.float32)   # (100, 18)

# One observation / action, must be correct shape (1, 18)
obs = obs_arr[0:1, :]
action = ort_sess.run(None, {'obs': obs})[0]  # run() returns a list of 1
print("np.shape(obs):", np.shape(obs))        # (1, 18)
print("np.shape(action):", np.shape(action))  # (1, 7)
print("obs:", obs)
print("action:", action)

# Multiple observations / actions
obs = obs_arr[0:10,:]
actions = ort_sess.run(None, {'obs': obs})[0]  # run() returns a list of 1
print("np.shape(obs):", np.shape(obs))          # (10, 18)
print("np.shape(actions):", np.shape(actions))  # (10, 7)
print("obs:", obs)
print("action:", action)
