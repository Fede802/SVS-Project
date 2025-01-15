import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..','utility')))
from stable_baselines3 import PPO
from env import env_cc, env_braking, env_braking2, env_following_breaking

env = env_following_breaking.GymEnv()
env.setup(ego_velocity=90, leader_velocity=70, min_distance_offset=14)


# env = env_braking()
# obs = env.setup(ego_velocity = 90, leader_velocity = 70, min_distance_offset = 14)

# env = env_braking2()
# obs = env.setup(ego_velocity = 90, leader_velocity = 70, min_distance_offset = 14)

trainedModel = PPO.load("./vehicle_controller/l_controller/model/env_following_braking/third_attempt/best_model", env=env)
obs = trainedModel.get_env().reset()
# trainedModel = PPO.load("working_braking_model", env=env)
# trainedModel = PPO.load("working_braking2_model", env=env)

while True:    
    action, _states = trainedModel.predict(obs, deterministic=True)
    obs, rewards, dones, info = trainedModel.get_env().step(action)