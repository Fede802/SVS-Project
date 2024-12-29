import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecMonitor, SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from env.env_braking import AccEnv
import carla_utility

# Create the environment
print(carla_utility.compute_security_distance(150))
seed = 42
env = AccEnv(seed)
env.reset(seed) #??? maybe useless

# Initialize the PPO model
model = PPO(
    "MlpPolicy",  # Use an MLP (feed-forward neural network) policy
    env,
    verbose=0,
    gamma=0.99,  # Discount factor
    learning_rate=0.00003,  # Learning rate
    n_steps=1024,  # Rollout buffer size
    batch_size=64,  # Batch size for training
    ent_coef=0.01,  # Entropy coefficient
    n_epochs=1000,
    seed=seed,
    tensorboard_log="./log"
    
)
# Train the PPO model
callback = EvalCallback(env, best_model_save_path='./',
                        log_path='./', eval_freq=1024,
                        deterministic=True, render=False)
model.learn(total_timesteps=1024*1000, callback=callback, progress_bar=True, log_interval=1)

# Save the final model
model.save("ppo_carla_model")
# Clean up the environment
env.close()