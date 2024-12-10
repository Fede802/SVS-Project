import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla
import debug_utility
import carla_utility as utility
import math

class AccEnv(gym.Env):
    
    TARGET_SPEED = 16.6667 #60 km/h
    SPAWN_POINT = carla.Location(x=2393, y=6000, z=167)
    SPAWN_YAW = -88.2
    VEHICLE_BP = 'vehicle.tesla.model3'

    def __init__(self):
        super(AccEnv, self).__init__()

        #Set up the CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(100.0)
        self.world = self.client.get_world()
        
        #TODO: Refactor Spectator
        self.spectator = self.world.get_spectator()
        self.spectator.set_transform(carla.Transform(self.SPAWN_POINT, carla.Rotation(yaw = self.SPAWN_YAW)))

        #Define the action for ego vehicle [throttle, break]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float64)

        #Define the observation space
        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, 0.0, -30.0, 0, 1]), # [min(distance), min(ego_speed), min(leader_speed), min(relative_speed), object detected, security_distance] in m/s
            high=np.array([50, 30.0, 30.0, 30.0, 1, 100]), # [max(distance, max(ego_speed), max(leader_speed), max(relative_speed), object detected, security_distance] in m/s
            dtype=np.float64
        )

        self.ego_vehicle = None
        self.leader_vehicle = None
        self.radar_sensor = None

        self.radar_data = {  # Default radar data
            "distance": 50,  # Default distance if no object is detected
            "relative_speed": 0.0,  # Default relative speed
            "object_detected": 0
        }


    def spawn_vehicles(self):
        #Spawn Ego Veichle
        spawn_point_ego = carla.Transform(self.SPAWN_POINT, carla.Rotation(yaw = self.SPAWN_YAW))
        self.ego_vehicle = utility.spawn_vehicle_bp_at(world=self.world, vehicle=self.VEHICLE_BP, spawn_point=spawn_point_ego)

        # Spawn point ego vehicle at random position
        random_distance = rnd.uniform(50.00, 100.0)
        
        #Spawn Leader Veichle
        self.leader_vehicle = utility.spawn_vehicle_bp_in_front_of(self.world, self.ego_vehicle, self.VEHICLE_BP, offset=random_distance)


    
    def reset(self, seed=42):
        # Destroy existing actors
        utility.destroy_all_vehicle_and_sensors(world=self.world)
        self.ego_vehicle = None
        self.leader_vehicle = None
        
        # Spawn vehicles
        self.spawn_vehicles()

        #Add radar sensor
        self.radar_sensor = utility.spawn_radar(world=self.world, attach_to=self.ego_vehicle, transform=carla.Transform(carla.Location(x=2.5, z=1.0), carla.Rotation(pitch=0)), range=50, vertical_fov=5, horizontal_fov=5)

        # Get data from radar sensor
        self.radar_sensor.listen(self._radar_callback)

        # Reset vehicle state
        random_ego_velocity = rnd.uniform(self.TARGET_SPEED - 5, self.TARGET_SPEED + 5)
        random_leader_velocity = rnd.uniform(self.TARGET_SPEED - 5, self.TARGET_SPEED + 5)
        self.ego_vehicle.set_target_velocity(carla.Vector3D(random_ego_velocity*math.cos(self.SPAWN_YAW), random_ego_velocity*math.sin(self.SPAWN_YAW), 0))
        self.leader_vehicle.set_target_velocity(carla.Vector3D(random_leader_velocity*math.cos(self.SPAWN_YAW), random_leader_velocity*math.sin(self.SPAWN_YAW),0))
        
        # Wait for simulation to progress
        self.world.tick()

        # Return initial observation
        return self._get_observation(), {}
    
    def _radar_callback(self, radar_data):
        distances = []
        velocities = []

        for detection in radar_data:
            debug_utility.draw_radar_point(self.radar_sensor, detection)
            distances.append(detection.depth) #Distance to detect object
            velocities.append(detection.velocity) #Velocity of detect object
        
        # Use the closest detected object as the relevant one for control
        if distances:
            self.radar_data["distance"] = min(distances)
            self.radar_data["relative_speed"] = velocities[np.argmin(distances)]
            self.radar_data["object_detected"] = 1
        else:
            self.radar_data["distance"] = 50.0
            self.radar_data["relative_speed"] = 0.0
            self.radar_data["object_detected"] = 0
        
    def step(self, action):
        
        action_value = action[0]
        
        if action_value > 0:
            throttle = action_value
            brake = 0.0
        else:
            brake = abs(action_value)
            throttle = 0.0
        
        throttle = np.clip(throttle, 0.0, 1.0)
        brake = np.clip(brake, 0.0, 1.0)

        # Apply action to throttle and brake
        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        self.ego_vehicle.apply_control(control)

        self.world.tick()
                
        observation = self._get_observation()
        reward = self._compute_reward(observation)
        done = self._check_done(observation)

        print(f"Throttle: {throttle}, Brake: {brake}, Reward: {reward}")
        

        return observation, reward, done, False, {}
    
    def _get_observation(self):
        
        distance = abs(self.radar_data["distance"])
        relative_speed = self.radar_data["relative_speed"]
        object_detected = self.radar_data["object_detected"]
        
        ego_speed = self.ego_vehicle.get_velocity().length()
        leader_speed = abs(ego_speed - relative_speed)
        security_distance = utility.compute_security_distance(ego_speed)
        
        return np.array([distance, abs(ego_speed), abs(leader_speed), relative_speed, object_detected, security_distance], dtype=np.float64)

 
    def _compute_reward(self, observation):
        distance, ego_speed, leader_speed, relative_speed, object_detected, security_distance = observation
        
        reward = 0
        
        if distance < 1:
            reward -= 1
        
        if ego_speed < 0.1:
            reward -= 0.5
        
        speed_error = abs(ego_speed - self.TARGET_SPEED)
        reward += 1 - abs(speed_error / self.TARGET_SPEED)
        
        if object_detected == 1 and distance < security_distance:
            reward -= 1 * (security_distance - distance) / security_distance
        
        return np.clip(reward, -1, 1)



    def _check_done(self, observation):
        distance, ego_speed, leader_speed, relative_speed, object_detected, security_distance = observation

        # Collision Detected
        if object_detected == 1 and distance <= 1:
            print("Collision")
            return True
        
        # Ego stop
        if ego_speed <= 0.1:
            print("Ego stopped")
            return True
        
        
        
        return False