import os
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from env import CarlaEnv

# Define paths to save models and logs
log_dir = "./ppo_logs/"
os.makedirs(log_dir, exist_ok=True)

env = CarlaEnv()

# Initialize the PPO model
model = PPO(
    "MlpPolicy",  # Use an MLP (feed-forward neural network) policy
    env,
    verbose=1,
    tensorboard_log=log_dir,  # Log data for TensorBoard visualization
    gamma=0.99,  # Discount factor
    learning_rate=0.0003,  # Learning rate
    n_steps=2048,  # Rollout buffer size
    batch_size=64,  # Batch size for training
    ent_coef=0.01,  # Entropy coefficient
)

# Set up a checkpoint callback to save models during training
checkpoint_callback = CheckpointCallback(
    save_freq=10000,  # Save every 10,000 steps
    save_path="./ppo_checkpoints/",
    name_prefix="ppo_carla"
)

# Train the PPO model
model.learn(total_timesteps=500000, callback=checkpoint_callback)

# Save the final model
model.save("ppo_carla_model")

# Clean up the environment
env.close()