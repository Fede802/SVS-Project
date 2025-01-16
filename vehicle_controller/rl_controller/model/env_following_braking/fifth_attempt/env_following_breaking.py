import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import time
import carla, debug_utility, carla_utility

class GymEnv(gym.Env):
    
    MIN_DISTANCE_SETTING = [7.0, 14.0, 21.0]
    MAX_VEHICLE_VELOCITY = 150.0 # In km/h
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
        self.is_env_for_following = False
    
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
        return spaces.Box(low=np.array([0.0, 0.0, self.MIN_DISTANCE_SETTING[0], -self.MAX_VEHICLE_VELOCITY / 3.6, 0., 0.0]), 
                          high=np.array([self.RADAR_RANGE, carla_utility.compute_security_distance(self.MAX_VEHICLE_VELOCITY) + self.MIN_DISTANCE_SETTING[-1], self.MIN_DISTANCE_SETTING[-1], self.MAX_VEHICLE_VELOCITY / 3.6, self.MAX_VEHICLE_VELOCITY / 3.6, self.MAX_VEHICLE_VELOCITY / 3.6]), dtype=np.float32)
    
    def __spawn_vehicles(self, ego_velocity):
        other_vehicle_offset = carla_utility.compute_security_distance(ego_velocity) + self.MIN_DISTANCE_SETTING[self.index_security_distance] + 10
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
        time.sleep(1)
        self.ego_vehicle.set_target_velocity(debug_utility.get_velocity_vector(ego_velocity / 3.6, self.SPAWN_POINT.rotation))
        return self._get_observation(), {}

    def setup(self, ego_velocity, leader_velocity, min_distance_offset):
        self.min_distance_offset = min_distance_offset
        return self.__reset(ego_velocity, leader_velocity)
    


    def reset(self, seed = None):
        print("---RESET---")
        seed is not None and rnd.seed(seed)
        self.is_env_for_following = not self.is_env_for_following
        random_ego_velocity = rnd.randint(self.MIN_VEHICLE_VELOCITY, self.MAX_VEHICLE_VELOCITY)
        random_leader_velocity = rnd.randint(2, random_ego_velocity) if self.is_env_for_following else 0
        print(f"Ego: {random_ego_velocity} km/h, Leader: {random_leader_velocity} km/h")
        self.min_distance_offset = self.MIN_DISTANCE_SETTING[self.index_security_distance]
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
        print(f"Distance: {int(obs[0])} m, Security Distance: {int(obs[1])} m, Min_Security_Distance: {int(obs[2])} m, Relative_Speed: {int(obs[3] * 3.6)} km/h, Ego_Velocity: {int(obs[4] * 3.6)} km/h, Absolute_Speed: {int(obs[5])} km/h, reward: {reward}")
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


        #Observation
        distance, security_distance, min_security_distance, relative_speed, ego_speed, absolute_speed = observation

        #Error
        distance_error = abs(security_distance - distance)
        min_security_distance_error = abs(distance - min_security_distance) / min_security_distance
        speed_error = abs(ego_speed - absolute_speed)

        #Conditions
        tolerance = 0.1 #1% error
        isObjectDetected = not distance >= self.RADAR_RANGE
        isEgoTooLong = distance > security_distance
        isEgoStop = ego_speed < 1.0
        isEgoInSecurityDistanceRange = security_distance - tolerance <= distance <= security_distance + tolerance
        isEgoTooClose = distance < security_distance - tolerance and not distance < min_security_distance

        #Reward
        reward = 0

        if isObjectDetected:
            #1 Case: Distance > Security Distance
            if isEgoTooLong:
                if isEgoStop:
                    reward -= 20
                else:
                    reward -= (distance_error / distance) + speed_error * 0.1 #TODO: Cancel speed error
            #2 Case: Security_Distance - Tolerance <= Distance <= Security_Distance + Tolerance
            elif isEgoInSecurityDistanceRange:
                reward += 10 - (speed_error * 0.5)
            #3 Case: Distance < Security_Distance and not Distance < Min_Security_Distance
            elif isEgoTooClose:
                reward -= (distance_error / security_distance) + speed_error * 0.1
            #4 Case: Distance < Min_Security_Distance
            else:
                if distance < self.MIN_DISTANCE_SETTING[0] - 4:
                    reward -= 10
                else:
                    reward -= 2 + min_security_distance_error #TODO: Check Distance: 13 m, Security Distance: 14 m, Min_Security_Distance: 14 m, Relative_Speed: 0 km/h, Ego_Velocity: 0 km/h, Absolute_Speed: 0 km/h, reward: -2.0466935969889164
                    #Distance: 23 m, Security Distance: 23 m, Min_Security_Distance: 7 m, Relative_Speed: -4 km/h, Ego_Velocity: 40 km/h, Absolute_Speed: 9 km/h, reward: 9.369238376617432

        return max(reward, -10)
            
            
    def _check_done(self, observation):
        distance, security_distance, min_security_distance, _, ego_speed, _ = observation
        isNoCarDetected = self.no_car_detected_counter >= 20
        isEgoInCollision = distance < self.MIN_DISTANCE_SETTING[0] - 4
        isEgoStoppedBefore = distance > security_distance and ego_speed < 1

        if isNoCarDetected:
            print("No detection")
            return True
        
        if isEgoInCollision:
            print("Collision")
            return True
        
        if isEgoStoppedBefore:
            print("Ego stopped before")
            return True
        
        if self.step_count >= 1024:
            print("Max step reached")
            return True
        
        return False