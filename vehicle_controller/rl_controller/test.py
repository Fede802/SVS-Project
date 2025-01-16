import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..','utility')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..','mqtt_service')))
from stable_baselines3 import PPO
from env import env_cc, env_braking

env = env_cc()
obs = env.setup(ego_velocity = 90, target_velocity = 110)

# env = env_braking()
# obs = env.setup(ego_velocity = 90, leader_velocity = 70, min_distance_offset = 14)

trainedModel = PPO.load("working_cc_model", env=env)
# trainedModel = PPO.load("working_braking_model", env=env)

while True:    
    action, _states = trainedModel.predict(obs, deterministic=True)
    obs, rewards, dones, info = trainedModel.get_env().step(action)