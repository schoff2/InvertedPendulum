# c:\workspace\InvertedPendulum\ReinforcementLearning\export_model.py
import torch
import os
import numpy as np
from stable_baselines3 import PPO

def export_to_header(model_path, output_file="DE10Nano-CPP/policy_weights.h"):
    """
    Exports the weights of a Stable Baselines3 PPO MlpPolicy to a C++ header file.
    """
    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    full_model_path = f"{model_path}.zip"
    if not os.path.exists(full_model_path):
        print(f"Error: Model not found at {full_model_path}")
        return

    print(f"Loading model from {full_model_path}...")
    model = PPO.load(model_path)
    
    # Access the policy network (Actor)
    # In SB3 PPO, the policy is split into a feature extractor (policy_net) and a final projection (action_net)
    policy_net = model.policy.mlp_extractor.policy_net
    action_net = model.policy.action_net
    
    layers = []
    
    # 1. Extract hidden layers from the feature extractor
    # These are usually Linear -> Tanh -> Linear -> Tanh ...
    for layer in policy_net:
        if isinstance(layer, torch.nn.Linear):
            layers.append({
                'weights': layer.weight.detach().cpu().numpy(),
                'biases': layer.bias.detach().cpu().numpy(),
                'activation': 'tanh'  # SB3 default for MlpPolicy
            })
            
    # 2. Extract the output layer (Action Net)
    # This projects the features to the action space (Linear, no activation for the mean)
    layers.append({
        'weights': action_net.weight.detach().cpu().numpy(),
        'biases': action_net.bias.detach().cpu().numpy(),
        'activation': 'linear'
    })
    
    print(f"Found {len(layers)} linear layers in the policy network.")
    print(f"Input Layer Dimension: {layers[0]['weights'].shape[1]}")

    # 3. Generate C++ Header
    with open(output_file, 'w') as f:
        f.write("// Auto-generated policy weights for DE10-Nano\n")
        f.write("// Generated from Stable Baselines3 PPO Model\n")
        f.write("#ifndef POLICY_WEIGHTS_H\n")
        f.write("#define POLICY_WEIGHTS_H\n\n")
        
        f.write(f"const int NUM_LAYERS = {len(layers)};\n\n")
        
        for i, layer in enumerate(layers):
            w = layer['weights']
            b = layer['biases']
            rows, cols = w.shape
            
            f.write(f"// Layer {i} ({layer['activation']}): {cols} inputs -> {rows} outputs\n")
            f.write(f"const int L{i}_IN = {cols};\n")
            f.write(f"const int L{i}_OUT = {rows};\n")
            
            # Write Weights as 2D array
            f.write(f"const float L{i}_W[{rows}][{cols}] = {{\n")
            for r in range(rows):
                f.write("    {")
                f.write(", ".join(f"{x:.8f}f" for x in w[r]))
                f.write("},\n")
            f.write("};\n")
            
            # Write Biases as 1D array
            f.write(f"const float L{i}_B[{rows}] = {{")
            f.write(", ".join(f"{x:.8f}f" for x in b))
            f.write("};\n\n")

        f.write("#endif // POLICY_WEIGHTS_H\n")
        
    print(f"Successfully exported weights to {output_file}")

if __name__ == "__main__":
    # Path to your trained model (without .zip extension)
    model_path = "models/PPO/best_model/best_model"
    export_to_header(model_path)
