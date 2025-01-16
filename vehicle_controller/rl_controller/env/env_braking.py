import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla, debug_utility, carla_utility

class GymEnv(gym.Env):
    
    MIN_DISTANCE_SETTING = [7.0, 14.0, 21.0]
    MAX_VEHICLE_VELOCITY = 150.0
    RADAR_RANGE = carla_utility.compute_security_distance(MAX_VEHICLE_VELOCITY) #try eventually + MIN_DISTANCE_SETTING[-1]
    SPAWN_POINT = carla.Transform(carla.Location(x=2388, y=6164, z=178), carla.Rotation(yaw = -88.2))

    def __init__(self):
        self.action_space = spaces.Box(
            low=-1.0, 
            high=1.0, 
            dtype=float
        ) # -1 max break, 1 max throttle
       
        self.observation_space = spaces.Box(
            low=np.array([self.MIN_DISTANCE_SETTING[0], 0.0]),
            high=np.array([carla_utility.compute_security_distance(self.MAX_VEHICLE_VELOCITY) + self.MIN_DISTANCE_SETTING[-1], self.RADAR_RANGE]),
            dtype=np.float32
        ) # [security_distance, nearest_obstacle_distance] in meters


    def __spawn_vehicles(self, ego_velocity):
        other_vehicle_offset = carla_utility.compute_security_distance(ego_velocity) + self.min_distance_offset
        self.ego_vehicle = carla_utility.spawn_vehicle_bp_at('vehicle.tesla.cybertruck', spawn_point=self.SPAWN_POINT)
        self.leader_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(self.ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=other_vehicle_offset)

    def __reset(self, ego_velocity, leader_velocity):
        carla_utility.destroy_all_vehicle_and_sensors() #to avoid spawning bugs
        self.radar_data = {"nearest_obstacle_distance": self.RADAR_RANGE}
        self.step_count = 0

        self.__spawn_vehicles(ego_velocity)
        self.radar_sensor = carla_utility.spawn_radar(self.ego_vehicle, range=self.RADAR_RANGE)
        self.radar_sensor.listen(self._radar_callback)

        self.ego_vehicle.set_target_velocity(debug_utility.get_velocity_vector(ego_velocity / 3.6, self.SPAWN_POINT.rotation))
        self.leader_vehicle.set_target_velocity(debug_utility.get_velocity_vector(leader_velocity / 3.6, self.SPAWN_POINT.rotation))
        
        return self._get_observation(), {}

    def setup(self, ego_velocity, leader_velocity, min_distance_offset):
        self.min_distance_offset = min_distance_offset
        return self.__reset(ego_velocity, leader_velocity)

    def reset(self, seed = None):
        seed is not None and rnd.seed(seed)
        random_ego_velocity = rnd.randint(0, self.MAX_VEHICLE_VELOCITY)
        random_leader_velocity = rnd.randint(0, random_ego_velocity)
        self.min_distance_offset = self.MIN_DISTANCE_SETTING[rnd.randrange(len(self.MIN_DISTANCE_SETTING))]
        return self.__reset(random_ego_velocity, random_leader_velocity)
        
    def _radar_callback(self, radar_data):
        distances = []
        velocities = []

        for detection in radar_data:
            if debug_utility.evaluate_point(self.radar_sensor, detection, 1, 0.8):
                distances.append(detection.depth)
                velocities.append(detection.velocity)
        
        # Use the closest detected object as the relevant one for control
        self.radar_data["nearest_obstacle_distance"] = min(distances) if distances else self.RADAR_RANGE

        
    def step(self, action):
        action = [max(action, 0), abs(min(action, 0))]
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=float(action[0]), brake=float(action[1])))
        obs = self._get_observation()  
        reward = self._compute_reward(obs)
        done = self._check_done(obs)
        print("security_distance: ", obs[0] ,"nearest_obstacle_distance: ", obs[1],"reward:", reward)    
        return obs, reward, done, False, {}
    
    def _get_observation(self):
        current_velocity = self.ego_vehicle.get_velocity().length() * 3.6
        security_distance = carla_utility.compute_security_distance(current_velocity) + self.min_distance_offset 
        return security_distance, self.radar_data["nearest_obstacle_distance"]

    def _compute_reward(self, observation):        
        obstacle_detected = observation[1] <= observation[0]
        if obstacle_detected:
            return -abs(observation[0] - observation[1])/10
        else:
            return 1/((observation[1] - observation[0])/10)
            
    def _check_done(self, observation):
        self.step_count += 1
        return self.step_count > 1024