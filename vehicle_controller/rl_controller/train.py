import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..','utility')))
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from env import env_cc, env_braking, env_braking2, env_braking3

# env = env_cc.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
# gym_env = env_braking.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
# gym_env = env_braking2.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
env = env_braking3.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay

n_steps = 1024
n_epochs = 500

model = PPO(
    "MlpPolicy", 
    env,
    learning_rate=0.00003,
    n_steps=n_steps, 
    batch_size=64,
    ent_coef=0.01,  
    n_epochs=n_epochs,
    seed=42,
    tensorboard_log="./log"  
)

callback = EvalCallback(env, best_model_save_path='./',
                        log_path='./', eval_freq=n_epochs,
                        deterministic=True, render=False)
model.learn(total_timesteps=n_steps*n_epochs, callback=callback, progress_bar=True, log_interval=1)
model.save("ppo_carla_model")

env.close()