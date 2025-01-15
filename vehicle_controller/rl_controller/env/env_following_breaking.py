import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import time
import carla, debug_utility, carla_utility

class GymEnv(gym.Env):
    
    MIN_DISTANCE_SETTING = [7.0, 14.0, 21.0]
    MAX_VEHICLE_VELOCITY = 60.0 # In km/h
    MIN_VEHICLE_VELOCITY = 30.0 # In km/h
    RADAR_RANGE = carla_utility.compute_security_distance(MAX_VEHICLE_VELOCITY)  + MIN_DISTANCE_SETTING[-1]
    SPAWN_POINT = carla.Transform(carla.Location(x=2388, y=6164, z=168), carla.Rotation(yaw = -88.2))
    SPAWN_POINT = carla.Transform(debug_utility.get_point_from_trasform(SPAWN_POINT, 300), SPAWN_POINT.rotation)

    def __init__(self):
        #Define action space and observation space
        self.action_space = self.__action_space()
        self.observation_space = self.__observation_space()
        self.index_security_distance = -1
        self.no_car_detected_counter = 0
    
    def __increment_index_security_distance(self):
        if self.index_security_distance >= self.MIN_DISTANCE_SETTING.index(max(self.MIN_DISTANCE_SETTING)):
            self.index_security_distance = 0
        else:
            self.index_security_distance += 1
        
    def __action_space(self):
        # -1 max break, 1 max throttle
        return spaces.Box(low=-1.0, high=1.0, dtype=np.float32)
    
    def __observation_space(self):
        # [nearest_obstacle_distance, security_distance, min_security_distance, relative_speed, ego_speed, absolute_speed] in meters
        return spaces.Box(low=np.array([0.0, 0.0, self.MIN_DISTANCE_SETTING[0], -self.MAX_VEHICLE_VELOCITY / 3.6, 0.0, 0.0]), 
                          high=np.array([self.RADAR_RANGE, carla_utility.compute_security_distance(self.MAX_VEHICLE_VELOCITY) + self.MIN_DISTANCE_SETTING[-1], self.MIN_DISTANCE_SETTING[-1], self.MAX_VEHICLE_VELOCITY / 3.6, self.MAX_VEHICLE_VELOCITY / 3.6, self.MAX_VEHICLE_VELOCITY / 3.6]), dtype=np.float32)
    
    def __spawn_vehicles(self, ego_velocity):
        other_vehicle_offset = carla_utility.compute_security_distance(ego_velocity) + self.min_distance_offset + 10
        self.ego_vehicle = carla_utility.spawn_vehicle_bp_at('vehicle.tesla.cybertruck', spawn_point=self.SPAWN_POINT)
        self.leader_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(self.ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=other_vehicle_offset)

    def __reset(self, ego_velocity, leader_velocity):
        carla_utility.destroy_all_vehicle_and_sensors() #to avoid spawning bugs
        self.radar_data = {"distance": self.RADAR_RANGE, "relative_speed": 0.0}
        self.step_count = 0
        self.ego_velocity = ego_velocity
        self.leader_velocity = leader_velocity
        self.__spawn_vehicles(ego_velocity)
        self.radar_sensor = carla_utility.spawn_radar(self.ego_vehicle, range=self.RADAR_RANGE)
        self.radar_sensor.listen(self._radar_callback)
        #self.leader_vehicle.apply_control(carla.VehicleControl())
        time.sleep(1)
        self.ego_vehicle.set_target_velocity(debug_utility.get_velocity_vector(ego_velocity / 3.6, self.SPAWN_POINT.rotation))
        return self._get_observation(), {}

    def setup(self, ego_velocity, leader_velocity, min_distance_offset):
        self.min_distance_offset = min_distance_offset
        return self.__reset(ego_velocity, leader_velocity)

    def reset(self, seed = None):
        print("---RESET---")
        seed is not None and rnd.seed(seed)
        random_ego_velocity = rnd.randint(self.MIN_VEHICLE_VELOCITY, self.MAX_VEHICLE_VELOCITY)
        random_leader_velocity = 0 #rnd.randint(0, random_ego_velocity)
        print(f"Ego: {random_ego_velocity} km/h, Leader: {random_leader_velocity} km/h")
        self.min_distance_offset = self.MIN_DISTANCE_SETTING[self.index_security_distance]#self.MIN_DISTANCE_SETTING[rnd.randrange(len(self.MIN_DISTANCE_SETTING))]
        self.__increment_index_security_distance()
        self.no_car_detected_counter = 0
        return self.__reset(random_ego_velocity, random_leader_velocity)
    
    def _radar_callback(self, radar_data):
        distances = []
        velocities = []
        ttc = []

        for detection in radar_data:
            if debug_utility.evaluate_point(self.radar_sensor, detection, 1, 0.8):
                #self.show_detection and debug_utility.draw_radar_point(self.radar_sensor, detection)
                distances.append(detection.depth)
                velocities.append(detection.velocity)
                ttc.append(detection.depth / detection.velocity if abs(detection.velocity) != 0 else float('inf'))
    
        self.radar_data["ttc"] = 1 / min(ttc) if len(ttc) > 0 else 0
        self.radar_data["distance"] = distances[ttc.index(min(ttc))] if len(distances) > 0 else self.RADAR_RANGE
        self.radar_data["relative_speed"] = velocities[ttc.index(min(ttc))] if len(velocities) > 0 else 0

        
    def step(self, action):
        action = [max(action, 0), abs(min(action, 0))]
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=float(action[0]), brake=float(action[1])))
        #print(f"{action[0]}, {action[1]}")
        
        self.leader_vehicle.set_target_velocity(debug_utility.get_velocity_vector(self.leader_velocity / 3.6, self.SPAWN_POINT.rotation))
        self.step_count += 1
        carla_utility.world.tick()
        
        obs = self._get_observation()  
        reward = self._compute_reward(obs)
        truncated = False
        done = self._check_done(obs)
        info = {}
        #print(f"{self.leader_vehicle.get_velocity().length() * 3.6} km/h")
        print(f"Distance: {obs[0]} m, Security Distance: {obs[1]} m, Min_Security_Distance: {obs[2]} m, Relative_Speed: {obs[3] * 3.6} km/h, Ego_Velocity: {obs[4] * 3.6} km/h, Absolute_Speed: {obs[5] * 3.6}, reward: {reward}")
        #print(f"Ego_Velocity: {obs[4] * 3.6} km/h, Absolute_Speed: {obs[5] * 3.6}, reward: {reward}")  
        return obs, reward, done, truncated, info
    
    def _get_observation(self):
        current_velocity = self.ego_vehicle.get_velocity().length() * 3.6
        security_distance = carla_utility.compute_security_distance(current_velocity) + self.MIN_DISTANCE_SETTING[self.index_security_distance]
        min_security_distance = self.MIN_DISTANCE_SETTING[self.index_security_distance]
        distance = self.radar_data["distance"]
        relative_speed = self.radar_data["relative_speed"]
        ego_velocity = self.ego_vehicle.get_velocity().length() #In m/s
        absolute_speed = ego_velocity + relative_speed #In m/s
        self.no_car_detected_counter += 1 if self.radar_data["distance"] == self.RADAR_RANGE else 0
        return np.array([distance, security_distance, min_security_distance, relative_speed, ego_velocity, abs(absolute_speed)], dtype=np.float32)

    def _compute_reward(self, observation):
        distance, security_distance, min_security_distance, _, ego_velocity, absolute_speed = observation
        distance = int(distance)
        security_distance = int(security_distance)
        ego_velocity = int(ego_velocity)
        tolerance = 1
        distance_penalty = 0
        speed_penalty = 0
        isObjectDetected = not distance >= self.RADAR_RANGE
        isEgoInSecurityDistanceRange = security_distance - tolerance <= distance <= security_distance + tolerance
        isEgoTooClose = distance < security_distance - tolerance and not (distance < self.MIN_DISTANCE_SETTING[0])
        isEgoInCollision = distance < self.MIN_DISTANCE_SETTING[0]
        isEgoStopped = ego_velocity < 1.0

        if isObjectDetected:
            #Distance penalty
            if distance > security_distance:
                if isEgoStopped:
                    distance_penalty -= 20.0
                    speed_penalty = 0
                else:
                    distance_penalty -= abs(security_distance - distance) / distance
                    speed_penalty = 0
            elif isEgoInSecurityDistanceRange: 
                distance_penalty += 10 - abs(security_distance - distance)
                speed_penalty = 0#speed_penalty -= abs(ego_velocity - absolute_speed) / max(absolute_speed, 0.1)
            elif isEgoTooClose: 
                distance_penalty -= abs(security_distance - distance) / security_distance
                speed_penalty = 0 #abs(ego_velocity - absolute_speed) / max(absolute_speed, 0.1)
            elif isEgoInCollision: 
                distance_penalty -= 20.0
                speed_penalty = 0
        else:
            distance_penalty = 0
            speed_penalty = 0
 

        return max(distance_penalty + speed_penalty, -20)
            
            
    def _check_done(self, observation):
        distance, security_distance, min_security_distance, _, ego_speed, _ = observation
        isNoCarDetected = self.no_car_detected_counter >= 20
        isEgoInCollision = distance < self.MIN_DISTANCE_SETTING[0]
        isEgoStoppedBefore = distance > security_distance and ego_speed < 1

        if isNoCarDetected:
            print("No detection")
            return True
        
        if isEgoInCollision:
            print("Collision")
            return True
        
        if isEgoStoppedBefore:
            print("Ego stopped")
            return True
        
        if self.step_count >= 1024:
            print("Max step reached")
            return True
        
        return False