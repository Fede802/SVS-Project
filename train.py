from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecMonitor, SubprocVecEnv
from env import AccEnv



# Trained environment
env = AccEnv()

# env = VecMonitor(SubprocVecEnv([AccEnv() for i in range(10)]))
# Initialize the PPO model
model = PPO(
    "MlpPolicy",  # Use an MLP (feed-forward neural network) policy
    env,
    verbose=1,
    gamma=0.99,  # Discount factor
    learning_rate=0.00003,  # Learning rate
    n_steps=2048,  # Rollout buffer size
    batch_size=64,  # Batch size for training
    ent_coef=0.01,  # Entropy coefficient
    n_epochs=100,
    tensorboard_log="./log"
    
)
# env = VecMonitor(SubprocVecEnv([AccEnv() for i in range(10)]), filename=None)
# Train the PPO model
model.learn(total_timesteps=10000, progress_bar=True)


# Save the final model
model.save("ppo_carla_model")
# Clean up the environment
env.close()