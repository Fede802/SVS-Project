import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..','utility')))
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from env import env_cc, env_braking, env_braking2, env_braking3, env_following_breaking

#env = env_cc.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
#env = env_braking.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
# gym_env = env_braking2.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
env = env_braking3.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
#env = env_following_breaking.GymEnv() #could be wrapped with VecMonitor to make additional check on observation coerence but okay
#env = Monitor(env)

#check_env(env)

n_steps = 1024
n_epochs = 500

model = PPO(
    "MlpPolicy",
    env,
    verbose=0,
    gamma=0.95,
    learning_rate=0.00003,
    batch_size=128,
    n_steps=n_steps,
    n_epochs=n_epochs,
    ent_coef=0.01,
    seed=1234,
    tensorboard_log="./log")

callback = EvalCallback(env, best_model_save_path='./',
                        log_path='./', eval_freq=n_epochs,
                        deterministic=True, render=False)

checkpoint_callback = CheckpointCallback(
  save_freq=n_steps * 50,
  save_path="./checkpoint/",
  name_prefix="rl_model",
  save_replay_buffer=True
)

model.learn(total_timesteps=n_steps*n_epochs, callback=callback, progress_bar=True, log_interval=1)
model.save("ppo_carla_model")

env.close()