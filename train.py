from stable_baselines3 import PPO
from env.env import AccEnv


# Trained environment
env = AccEnv()


# Initialize the PPO model
model = PPO(
    "MlpPolicy",  # Use an MLP (feed-forward neural network) policy
    env,
    verbose=1,
    gamma=0.99,  # Discount factor
    learning_rate=0.0003,  # Learning rate
    n_steps=2048,  # Rollout buffer size
    batch_size=64,  # Batch size for training
    ent_coef=0.01,  # Entropy coefficient
    n_epochs=100,
    tensorboard_log="./log"
    
)
# Train the PPO model
model.learn(total_timesteps=100000, progress_bar=True)


# Save the final model
model.save("ppo_carla_model")
# Clean up the environment
env.close()