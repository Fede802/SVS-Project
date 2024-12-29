import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
import gymnasium.wrappers.atari_preprocessing
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecMonitor, SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from env.env import AccEnv
from env.env_cc import AccEnvConstantSpeed
import gymnasium 


# Trained environment
# env = AccEnv()
env = AccEnvConstantSpeed()

env.reset()
# env = gymnasium.wrappers.atari_preprocessing.AtariPreprocessing(env, frame_skip = 4)

# env = VecMonitor(SubprocVecEnv([AccEnv() for i in range(10)]))

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
    n_epochs=500,
    seed=42,
    tensorboard_log="./log"
    
)
# env = VecMonitor(SubprocVecEnv([AccEnv() for i in range(10)]), filename=None)
# Train the PPO model
callback = EvalCallback(env, best_model_save_path='./',
                        log_path='./', eval_freq=1000,
                        deterministic=True, render=False)
model.learn(total_timesteps=1024*500, callback=callback, progress_bar=True, log_interval=1)


# Save the final model
model.save("ppo_carla_model")
# Clean up the environment
env.close()