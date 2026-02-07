import gymnasium as gym
from gymnasium import spaces
import numpy as np
import cv2
import math

class InvertedPendulumEnv(gym.Env):
    """
    Custom Gymnasium Environment for the Inverted Pendulum with Domain Randomization.
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super(InvertedPendulumEnv, self).__init__()

        # --- Base Physical Constants ---
        # These are the nominal values for your 8mm steel rod + washers setup.
        self.base_L = 0.35        # Effective Length (meters) - Match hardware (MPC.cpp)
        self.base_g = 9.81        # Gravity
        self.dt = 0.005           # 5ms control loop
        
        # --- Base Friction Parameters (Newly Added) ---
        self.base_pole_damping = 0.05  # Pivot friction
        # Cart friction removed because closed-loop stepper rejects it
        # self.base_cart_friction = 0.02 
        
        # --- Current Episode Parameters ---
        # These will be overwritten by Domain Randomization on every reset
        self.L = self.base_L
        self.g = self.base_g
        self.pole_damping = self.base_pole_damping
        # self.cart_friction = self.base_cart_friction

        # --- Hardware Constraints (NEMA 23 / Rail) ---
        self.steps_per_meter = 40000.0
        self.max_accel_steps = 400000.0
        self.max_speed_steps = 100000.0
        self.total_track_steps = 26120
        self.safety_margin_steps = 300
        
        self.max_accel = self.max_accel_steps / self.steps_per_meter
        self.max_speed = self.max_speed_steps / self.steps_per_meter
        half_track = self.total_track_steps / 2.0
        self.x_limit = (half_track - self.safety_margin_steps) / self.steps_per_meter

        # --- Action & Observation Spaces ---
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        
        high = np.array([
            self.x_limit * 2.0,
            self.max_speed * 2.0,
            np.pi * 2.0,
            np.finfo(np.float32).max
        ], dtype=np.float32)
        
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)
        
        self.render_mode = render_mode
        self.state = None
        
        # Rendering config
        self.screen_width = 1000
        self.screen_height = 1000
        self.current_step = 0
        self.frame_skip = 4 
        self.max_steps = 500 
        
        # Sim2Real: Observation Noise (x, x_dot, theta, theta_dot)
        self.obs_noise_std = np.array([0.002, 0.01, 0.002, 0.05], dtype=np.float32)

        # Store base values for domain randomization (must be after all variables are defined)
        self.base_max_accel = self.max_accel
        self.base_dt = self.dt
        self.base_obs_noise_std = self.obs_noise_std.copy()

    def _randomize_domain(self):
        """
        DOMAIN RANDOMIZATION:
        Sample physics parameters from a uniform distribution to make
        the RL agent robust to the Sim2Real reality gap.
        """
        # Randomize Effective Length by +/- 10% (around correct 0.35m value)
        self.L = np.random.uniform(self.base_L * 0.9, self.base_L * 1.1)

        # Randomize Gravity by +/- 5% (forces robust stabilization)
        self.g = np.random.uniform(self.base_g * 0.95, self.base_g * 1.05)

        # Randomize Pole Damping by +/- 50% (pivot bearing friction is hard to model)
        self.pole_damping = np.random.uniform(
            self.base_pole_damping * 0.5,
            self.base_pole_damping * 1.5
        )

        # NEW: Randomize acceleration limits by +/- 20%
        # Hardware uses different limits for swing-up vs stabilization
        self.max_accel = np.random.uniform(
            self.base_max_accel * 0.8,
            self.base_max_accel * 1.2
        )

        # NEW: Randomize observation noise levels by +/- 30%
        noise_scale = np.random.uniform(0.7, 1.3)
        self.obs_noise_std = self.base_obs_noise_std * noise_scale

        # NEW: Randomize timestep by +/- 5% (models timing jitter)
        self.dt = np.random.uniform(self.base_dt * 0.95, self.base_dt * 1.05)

    def _apply_physics(self, action):
        x, x_dot, theta, theta_dot = self.state

        # Convert normalized action to physical target speed
        target_speed = np.clip(action[0], -1.0, 1.0) * self.max_speed

        # Calculate required acceleration to reach target speed
        # CRITICAL FIX: Model hardware acceleration limits properly
        needed_accel = (target_speed - x_dot) / self.dt

        # Apply hardware acceleration limits
        effective_accel = np.clip(needed_accel, -self.max_accel, self.max_accel)

        # 1. Cart Dynamics - Integrate acceleration to velocity
        x_dot_next = x_dot + effective_accel * self.dt
        x_dot_next = np.clip(x_dot_next, -self.max_speed, self.max_speed)

        # Integrate velocity to position
        x_next = x + x_dot_next * self.dt

        # Store for reward calculation
        self.last_effective_accel = effective_accel

        # 2. Pendulum Dynamics (Now using randomized L, g, and damping)
        damping_force = self.pole_damping * theta_dot
        theta_acc = (self.g / self.L) * np.sin(theta) + \
                    (1.0 / self.L) * effective_accel * np.cos(theta) - damping_force

        theta_dot_next = theta_dot + theta_acc * self.dt
        theta_next = theta + theta_dot_next * self.dt

        # Normalize theta to [-pi, pi]
        theta_next = (theta_next + np.pi) % (2 * np.pi) - np.pi

        self.state = np.array([x_next, x_dot_next, theta_next, theta_dot_next], dtype=np.float32)

    def _calculate_reward(self):
        x, x_dot, theta, theta_dot = self.state

        # Normalize theta to [-pi, pi]
        theta_normalized = (theta + np.pi) % (2 * np.pi) - np.pi

        # Check termination
        terminated = bool(x < -self.x_limit or x > self.x_limit)

        # Calculate energy (mimics hardware energy shaping strategy)
        # Normalized energy: e = KE/(g*L) + PE where PE = cos(theta)
        current_ke = 0.5 * (self.L * theta_dot) ** 2 / (self.g * self.L)
        current_pe = np.cos(theta_normalized)  # -1 (down) to 1 (up)
        current_energy = current_ke + current_pe
        target_energy = 1.0  # Energy at upright position

        # Two-phase reward matching hardware control strategy
        TRANSITION_ANGLE = np.radians(20)  # Match MPC.cpp transition threshold

        if abs(theta_normalized) < TRANSITION_ANGLE and abs(theta_dot) < 8.0:
            # STABILIZATION PHASE - Reward balancing upright
            r_theta = np.cos(theta_normalized) + 1.0  # 0 to 2
            r_omega = -0.1 * (theta_dot ** 2)  # Penalize angular velocity
            r_x = -0.5 * (x / self.x_limit) ** 2  # Keep centered
            r_x_dot = -0.01 * (x_dot ** 2)  # Minimize cart velocity
            r_control = -0.001 * (self.last_effective_accel / self.max_accel) ** 2

            reward = r_theta + r_omega + r_x + r_x_dot + r_control

        else:
            # SWING-UP PHASE - Reward energy shaping progress
            energy_error = abs(current_energy - target_energy)
            r_energy = -energy_error  # Minimize energy error
            r_height = current_pe  # -1 to 1, rewards getting higher
            r_x = -0.1 * (x / self.x_limit) ** 2  # Light centering
            r_control = -0.001 * (self.last_effective_accel / self.max_accel) ** 2

            reward = 2.0 * r_energy + 1.0 * r_height + r_x + r_control

        if terminated:
            reward = -10.0

        return float(reward), terminated

    def step(self, action):
        total_reward = 0.0
        terminated = False
        truncated = False

        for _ in range(self.frame_skip):
            self._apply_physics(action)
            step_reward, term = self._calculate_reward()
            total_reward += step_reward
            if term:
                terminated = True
                break

        self.current_step += 1
        truncated = bool(self.current_step >= self.max_steps)

        # Sim2Real: Add noise to observation (FIXED: moved before render/return)
        noisy_obs = self.state + np.random.normal(0, self.obs_noise_std)

        if self.render_mode == "human":
            self.render()

        return noisy_obs.astype(np.float32), total_reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # TRIGGER DOMAIN RANDOMIZATION ON RESET
        self._randomize_domain()
        
        start_mode = options.get("start_mode", "swing_up") if options else "swing_up"
        if start_mode == "upright":
            theta_init = np.random.uniform(-0.1, 0.1)
        else:
            theta_init = np.pi + np.random.uniform(-0.1, 0.1)
            
        self.state = np.array([0.0, 0.0, theta_init, 0.0], dtype=np.float32)
        # Sim2Real: Randomize initial cart state slightly
        x_init = np.random.uniform(-0.05, 0.05)
        x_dot_init = np.random.uniform(-0.05, 0.05)
        
        self.state = np.array([x_init, x_dot_init, theta_init, 0.0], dtype=np.float32)
        self.current_step = 0
        self.last_effective_accel = 0.0
        if self.render_mode == "human":
            self.render()
        return self.state, {}

    def render(self):
        if self.render_mode is None:
            return
            
        # Create a white background
        img = np.ones((self.screen_height, self.screen_width, 3), dtype=np.uint8) * 255
        
        x, _, theta, _ = self.state
        
        # Scaling
        scale = self.screen_width / (self.x_limit * 3)
        center_x = self.screen_width // 2
        cart_y = self.screen_height // 2 + 50
        
        # Draw Rail
        rail_left = int(center_x - self.x_limit * scale)
        rail_right = int(center_x + self.x_limit * scale)
        cv2.line(img, (rail_left, cart_y), (rail_right, cart_y), (0, 0, 0), 2)
        
        # Draw Cart
        cart_x_px = int(center_x + x * scale)
        cv2.rectangle(img, (cart_x_px - 20, cart_y - 10), (cart_x_px + 20, cart_y + 10), (0, 0, 0), -1)
        
        # Draw Pole
        # theta=0 is Up, theta>0 is Left (CCW)
        pole_len_px = int(self.L * scale)
        tip_x = int(cart_x_px - pole_len_px * np.sin(theta))
        tip_y = int(cart_y - pole_len_px * np.cos(theta))
        
        cv2.line(img, (cart_x_px, cart_y), (tip_x, tip_y), (0, 0, 255), 4)
        
        # Text Info
        info_str = f"time: {self.current_step*self.dt*self.frame_skip:.2f} s | x: {x:.2f}m | theta: {np.degrees(theta):.1f} deg"
        cv2.putText(img, info_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 50, 50), 2)
        
        if self.render_mode == "human":
            cv2.imshow("Inverted Pendulum Sim", img)
            cv2.waitKey(1)
        
        return img if self.render_mode == "rgb_array" else None

    def close(self):
        if self.render_mode == "human":
            cv2.destroyAllWindows()

# --- Quick Test ---
if __name__ == "__main__":
    # Example usage
    env = InvertedPendulumEnv(render_mode="human")
    obs, _ = env.reset(options={"start_mode": "swing_up"})
    
    print("Simulation started. Press Ctrl+C to stop.")
    try:
        while True:
            # Random action for testing
            action = env.action_space.sample()
            
            obs, reward, terminated, truncated, info = env.step(action)
            
            if terminated or truncated:
                obs, _ = env.reset(options={"start_mode": "swing_up"})
    except KeyboardInterrupt:
        print("\nStopped.")
        env.close()
