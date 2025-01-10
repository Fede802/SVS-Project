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
            dtype=np.float32
        ) # -1 max break, 1 max throttle
       
        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, self.MIN_DISTANCE_SETTING[0]]),
            high=np.array([float('inf'), self.RADAR_RANGE, carla_utility.compute_security_distance(self.MAX_VEHICLE_VELOCITY) + self.MIN_DISTANCE_SETTING[-1]]),
            dtype=np.float32
        ) # [1/ttc, nearest_obstacle_distance, security_distance] in meters


    def __spawn_vehicles(self, ego_velocity):
        other_vehicle_offset = carla_utility.compute_security_distance(ego_velocity) + self.min_distance_offset
        self.ego_vehicle = carla_utility.spawn_vehicle_bp_at('vehicle.tesla.cybertruck', spawn_point=self.SPAWN_POINT)
        self.leader_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(self.ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=other_vehicle_offset)

    def __reset(self, ego_velocity, leader_velocity):
        carla_utility.destroy_all_vehicle_and_sensors() #to avoid spawning bugs
        self.radar_data = {"ttc": float('inf'), "distance": self.RADAR_RANGE}
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
        print("---RESET---")
        seed is not None and rnd.seed(seed)
        random_ego_velocity = rnd.randint(0, self.MAX_VEHICLE_VELOCITY)
        random_leader_velocity = rnd.randint(0, random_ego_velocity)
        self.min_distance_offset = self.MIN_DISTANCE_SETTING[rnd.randrange(len(self.MIN_DISTANCE_SETTING))]
        return self.__reset(random_ego_velocity, random_leader_velocity)
    
    def _radar_callback(self, radar_data):
        distances = []
        velocities = []
        ttc = []

        for detection in radar_data:
            if debug_utility.evaluate_point(self.radar, detection, self.radar_detection_h_radius, self.radar_detection_v_radius):
                self.show_detection and debug_utility.draw_radar_point(self.radar_sensor, detection)
                distances.append(detection.depth)
                velocities.append(detection.velocity)
                ttc.append(detection.depth / detection.velocity if abs(detection.velocity) != 0 else float('inf'))
    
        self.radar_data["ttc"] = 1/min(ttc) if len(ttc) > 0 else float('inf')
        self.radar_data["distance"] = distances[ttc.index(min(ttc))] if len(distances) > 0 else self.RADAR_RANGE
        #self.radar_data["relative_speed"] = velocities[ttc.index(min(ttc))] if len(velocities) > 0 else float('inf')

        
    def step(self, action):
        action = [max(action, 0), abs(min(action, 0))]
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=float(action[0]), brake=float(action[1])))
        obs = self._get_observation()  
        reward = self._compute_reward(obs, float(action[1]))
        truncated = False
        done = self._check_done(obs)
        info = {}
        print(f"1/ttc: {obs[0]}, distances: {obs[1]}, security_distance: {obs[2]}, reward: {reward}")    
        return obs, reward, done, truncated, info
    
    def _get_observation(self):
        current_velocity = self.ego_vehicle.get_velocity().length() * 3.6
        security_distance = carla_utility.compute_security_distance(current_velocity) + self.min_distance_offset 
        return np.array([self.radar_data["ttc"], self.radar_data["distance"], security_distance], dtype=np.float32)

    def _compute_reward(self, observation, brake_intensity):
        ttc, distance, security_distance = observation
        ttc_penalty = 0
        security_distance_penalty = 0
        brake_intensity_penalty = 0

        #TTC <= 1.0 collision
        #if ttc >= 1.0: ttc_penalty += -10 * (ttc - 1.0)
        #else: ttc_penalty = 0

        #No maintain security distance
        if distance < security_distance: security_distance_penalty += abs(security_distance - distance) / security_distance
        elif security_distance - 0.2 <= distance <= security_distance + 0.2: security_distance_penalty += 10 - abs(security_distance - distance)
        else: security_distance_penalty = 0

        if brake_intensity > 0.8: brake_intensity_penalty += abs(brake_intensity - brake_intensity)

        return ttc_penalty + security_distance_penalty + brake_intensity_penalty
            
            
    def _check_done(self, observation):
        ttc, distance, security_distance = observation
        self.step_count += 1
        return False #self.step_count > 1024 or distance <= 1.0