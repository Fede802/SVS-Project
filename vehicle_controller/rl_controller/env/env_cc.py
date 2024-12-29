import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla, carla_utility, debug_utility

class GymEnv(gym.Env):
    
    MIN_TARGET_VELOCITY = 30.0
    MAX_TARGET_VELOCITY = 150.0
    MAX_SPAWN_VELOCITY = 180.0
    MAX_VEHICLE_VELOCITY = 300.0
    SPAWN_POINT = carla.Transform(carla.Location(x=2388, y=6164, z=178), carla.Rotation(yaw = -88.2))

    def __init__(self):
        self.action_space = spaces.Box(
            low=-1.0, 
            high=1.0, 
            dtype=float
        ) # -1 max break, 1 max throttle
       
        self.observation_space = spaces.Box(
            low=np.array([self.MIN_TARGET_VELOCITY, 0.0]), 
            high=np.array([self.MAX_TARGET_VELOCITY, self.MAX_VEHICLE_VELOCITY]),
            dtype=np.float32
        ) # [target_velocity, ego_velocity]

    def __spawn_vehicles(self):
        self.ego_vehicle = carla_utility.spawn_vehicle_bp_at('vehicle.tesla.cybertruck', self.SPAWN_POINT)
        # probably useless
        # ego_offset = rnd.randint(0, 100)
        # self.ego_vehicle = carla_utility.spawn_vehicle_bp_at('vehicle.tesla.cybertruck', spawn_point=carla.Transform(debug_utility.get_point_from_trasform(self.SPAWN_POINT, ego_offset), self.SPAWN_POINT.rotation))

    def __reset(self, ego_velocity):
        carla_utility.destroy_all_vehicle_and_sensors() #to avoid spawning bugs
        self.step_count = 0
        self.__spawn_vehicles()
        self.ego_vehicle.set_target_velocity(debug_utility.get_velocity_vector(ego_velocity / 3.6, self.SPAWN_POINT.rotation))
        return self._get_observation(), {}
    
    def setup(self, ego_velocity, target_velocity):
        self.target_speed = target_velocity
        return self.__reset(ego_velocity)

    def reset(self, seed = None):
        seed is not None and rnd.seed(seed)
        random_ego_velocity = rnd.randint(0, self.MAX_SPAWN_VELOCITY)
        self.target_speed = rnd.randint(self.MIN_TARGET_VELOCITY, self.MAX_TARGET_VELOCITY)
        return self.__reset(random_ego_velocity)
        
    def step(self, action):
        action = [max(action, 0), abs(min(action, 0))]
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=float(action[0]), brake=float(action[1])))
        obs = self._get_observation()  
        reward = self._compute_reward(obs)
        done = self._check_done(obs)
        print("target_velocity: ", obs[0], "ego_velocity: ",obs[1], "reward:", reward)    
        return obs, reward, done, False, {}
    
    def _get_observation(self):
        ego_speed = self.ego_vehicle.get_velocity().length() * 3.6
        return self.target_speed, ego_speed

    def _compute_reward(self, observation):
        return -abs(observation[0] - observation[1])/10
        
    def _check_done(self, observation):
        self.step_count += 1
        return self.step_count > 1024      