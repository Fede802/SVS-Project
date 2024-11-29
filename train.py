import os
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from env.env import CarlaEnv

# Trained environmentw
env = CarlaEnv()

check_env(env) # check environment


# Initialize the PPO model
model = PPO(
    "MlpPolicy",  # Use an MLP (feed-forward neural network) policy
    env,
    verbose=1,
    gamma=0.99,  # Discount factor
    learning_rate=0.0001,  # Learning rate
    n_steps=2048,  # Rollout buffer size
    batch_size=64,  # Batch size for training
    ent_coef=0.01,  # Entropy coefficient
)
# Train the PPO model
model.learn(total_timesteps=500000)

# Save the final model
model.save("ppo_carla_model")
# Clean up the environment
env.close()