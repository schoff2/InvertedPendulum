import gymnasium as gym
from gymnasium.wrappers import FrameStackObservation, FlattenObservation
import os
from stable_baselines3 import PPO
from simulation import InvertedPendulumEnv
import time

def main():
    # Path to the saved model
    # SB3 adds .zip automatically when saving, so we look for that
    model_path = "models/PPO/best_model/best_model"
    
    if not os.path.exists(f"{model_path}.zip"):
        print(f"Error: Model not found at {model_path}.zip")
        print("Please run train_ppo.py first to generate the model.")
        return

    print(f"Loading model from {model_path}...")
    # Load the trained agent
    model = PPO.load(model_path)

    # Create environment with human rendering enabled
    env = InvertedPendulumEnv(render_mode="human")
    dt = env.dt
    frame_skip = env.frame_skip
    env = FlattenObservation(FrameStackObservation(env, stack_size=4))
    
    episodes = 10
    print(f"Starting evaluation for {episodes} episodes...")
    print("Press Ctrl+C to stop early.")

    try:
        for ep in range(episodes):
            # Start in swing_up mode to test the full capability (hanging down)
            # Change to "upright" to test stabilization only
            obs, _ = env.reset(options={"start_mode": "swing_up"})
            
            terminated = False
            truncated = False
            score = 0
            step = 0
            
            while not (terminated or truncated):
                step_start_time = time.time()
                
                # Predict action
                # deterministic=True ensures the agent uses the optimal policy it learned
                action, _ = model.predict(obs, deterministic=True)
                
                obs, reward, terminated, truncated, info = env.step(action)
                score += reward
                step += 1

                # sleep to match env.dt
                sleep_time = dt*frame_skip - (time.time() - step_start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

                
            print(f"Episode {ep+1}/{episodes} | Score: {score:.2f} | Steps: {step}")
            
    except KeyboardInterrupt:
        print("\nEvaluation stopped by user.")
    finally:
        env.close()

if __name__ == "__main__":
    main()
