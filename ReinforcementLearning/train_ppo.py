import gymnasium as gym
from gymnasium.wrappers import FlattenObservation, FrameStackObservation
import os
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from simulation import InvertedPendulumEnv

def main():
    # 1. Setup paths for logging and saving
    models_dir = "models/PPO"
    log_dir = "logs"
    os.makedirs(models_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    # 2. Initialize Environments
    # Wrap in Monitor to log episode stats (reward, length) to Tensorboard
    env = InvertedPendulumEnv()
    env = Monitor(env, log_dir)
    env = FlattenObservation(FrameStackObservation(env, stack_size=4))
    # Create a separate environment for evaluation
    eval_env = InvertedPendulumEnv()
    eval_env = Monitor(eval_env)
    eval_env = FlattenObservation(FrameStackObservation(eval_env, stack_size=4))
    
    # 3. Configure the EvalCallback
    # This will evaluate the agent every 10,000 steps
    eval_callback = EvalCallback(
        eval_env, 
        best_model_save_path=os.path.join(models_dir, "best_model"),
        log_path=log_dir, 
        eval_freq=10000,
        deterministic=True, 
        render=False
    )
    
    # Validate the environment to ensure it complies with Gymnasium API
    print("Checking environment compatibility...")
    check_env(env.unwrapped)
    print("Environment check passed.")

    # 4. Initialize PPO Agent
    # Custom policy network architecture: 2 hidden layers of 256 units
    # This provides more capacity for the agent to learn complex dynamics
    policy_kwargs = dict(
        activation_fn=nn.Tanh,
        net_arch=dict(pi=[256, 256], vf=[256, 256])
    )

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log=log_dir,
        learning_rate=5e-4,        # Increased for faster learning (was 3e-4)
        n_steps=4096,              # Increased horizon for long-term planning (was 2048)
        batch_size=512,            # Keep same (4096/512 = 8 minibatches)
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,             # Increased for more exploration (was 0.005)
        policy_kwargs=policy_kwargs,
        device="auto"              # Uses GPU if available, otherwise CPU
    )

    # 5. Train the Agent
    # 3M steps is usually sufficient for this task with the larger network
    TIMESTEPS = 3_000_000
    print(f"Starting training for {TIMESTEPS:,} timesteps...")
    print(f"To monitor training, run: tensorboard --logdir={log_dir}")
    model.learn(total_timesteps=TIMESTEPS,
                callback=eval_callback,
                progress_bar=True)

    # 6. Save the Final Model
    model_path = os.path.join(models_dir, "ppo_inverted_pendulum_final")
    model.save(model_path)
    print(f"Model saved to {model_path}.zip")

    # 7. Evaluate and Visualize
    print("Visualizing trained model...")
    # Re-initialize environment with render_mode="human" for visualization
    env_eval = InvertedPendulumEnv(render_mode="human")
    env_eval = FlattenObservation(FrameStackObservation(env_eval, stack_size=4))
    
    episodes = 5
    for ep in range(episodes):
        # Start in swing_up mode to test the full capability
        obs, _ = env_eval.reset(options={"start_mode": "swing_up"})
        terminated = False
        truncated = False
        score = 0
        
        while not (terminated or truncated):
            # Predict action (deterministic=True gives the best learned behavior)
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env_eval.step(action)
            score += reward
            
        print(f"Episode {ep+1} Score: {score:.2f}")
    
    env_eval.close()

if __name__ == "__main__":
    main()
