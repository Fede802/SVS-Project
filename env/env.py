import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla
import debug_utility
import carla_utility as utility
import math

class AccEnv(gym.Env):
    
    TARGET_SPEED = 90 #90 km/h
    SPAWN_POINT = carla.Location(x=2393, y=6000, z=167)
    SPAWN_YAW = -88.2
    VEHICLE_BP = 'vehicle.tesla.model3'

    def __init__(self):
        super(AccEnv, self).__init__()

        #Set up the CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(100.0)
        self.world = self.client.get_world()
        
        self.spectator = self.world.get_spectator()

        #Define the action for ego vehicle [throttle, break]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float64)

        #Define the observation space
        self.observation_space = spaces.Box(
            # [min(ttc), min(ego_speed), min(leader_speed), security_distance] in km/h
            low=np.array([0.0, 0.0, 0]),
            # [max(ttc), max(ego_speed), max(leader_speed), security_distance] in km/h
            high=np.array([np.inf, 130.0, 170]),
            dtype=np.float64
        )

        self.ego_vehicle = None
        self.leader_vehicle = None
        self.radar_sensor = None

        self.radar_data = {  # Default radar data
            "ttc": np.inf,  # Default distance if no object is detected
        }

    def spawn_vehicles(self):
        #Spawn Ego Veichle
        spawn_point_ego = carla.Transform(self.SPAWN_POINT, carla.Rotation(yaw = self.SPAWN_YAW))
        self.ego_vehicle = utility.spawn_vehicle_bp_at(world=self.world, vehicle=self.VEHICLE_BP, spawn_point=spawn_point_ego)
        utility.move_spectator_to(self.spectator, self.ego_vehicle.get_transform())

        # Spawn point ego vehicle at random position
        random_distance = np.random.randint(50, 100)
        
        #Spawn Leader Veichle
        self.leader_vehicle = utility.spawn_vehicle_bp_in_front_of(self.world, self.ego_vehicle, self.VEHICLE_BP, offset=random_distance)
        
        # Reset vehicle state
        #random_ego_velocity = (self.TARGET_SPEED - 1) * np.random.sample() + 1
        #random_leader_velocity = (self.TARGET_SPEED - 1) * np.random.sample() + 1
        self.ego_vehicle.set_target_velocity(carla.Vector3D(0*math.cos(self.SPAWN_YAW), self.TARGET_SPEED*math.sin(self.SPAWN_YAW), 0))
        self.leader_vehicle.set_target_velocity(carla.Vector3D(0*math.cos(self.SPAWN_YAW), self.TARGET_SPEED*math.sin(self.SPAWN_YAW),0))


    
    def reset(self, seed=42):
        # Destroy existing actors
        utility.destroy_all_vehicle_and_sensors(world=self.world)
        self.ego_vehicle = None
        self.leader_vehicle = None
        
        # Spawn vehicles
        self.spawn_vehicles()

        #Add radar sensor
        radar_range_offset = 20
        radar_offset = 180
        self.radar_sensor = utility.spawn_radar(world=self.world, attach_to=self.ego_vehicle, transform=carla.Transform(carla.Location(x=2.5, z=1.0), carla.Rotation(pitch=0)), range=radar_offset)

        # Get data from radar sensor
        self.radar_sensor.listen(self._radar_callback)

        
        # Wait for simulation to progress
        self.world.tick()

        # Return initial observation
        return self._get_observation(), {}
    
    def _radar_callback(self, radar_data):
        distances = []
        velocities = []

        for detection in radar_data:
            if debug_utility.evaluate_point(self.radar_sensor, detection, 1, 0.8):
                debug_utility.draw_radar_point(self.radar_sensor, detection)
                distances.append(detection.depth) #Distance to detect object
                velocities.append(detection.velocity) #Velocity of detect object
        
        # Use the closest detected object as the relevant one for control
        if distances:
            self.radar_data["ttc"] = min(distances) / max(0.1, abs(velocities[np.argmin(distances)])) 
        else:
            self.radar_data["ttc"] = np.inf
        
    def step(self, action):
        
        action_value = action[0]
        
        if action_value > 0:
            throttle = action_value
            brake = 0.0
        else:
            brake = abs(action_value)
            throttle = 0.0

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
        
        ttc = self.radar_data["ttc"]
        ego_speed = self.ego_vehicle.get_velocity().length()
        
        security_distance = utility.compute_security_distance(ego_speed)
        
        print(f"ttc: {ttc}, ego_speed: {ego_speed} km/h, security_distance: {security_distance}")
        
        return np.array([ttc, abs(ego_speed), security_distance], dtype=np.float64)

 
    def _compute_reward(self, observation):
        ttc, ego_speed, _ = observation
        
        reward = 0
        
        if self.TARGET_SPEED - 1 <= ego_speed <= self.TARGET_SPEED + 1: #Okay target speed
            reward += 1
        else:
            reward -= 1
            
        if ttc > 2.5:
            reward += 0.1 * (ttc)
        else:
            reward -= 0.5 * (ttc)
        
        
        return np.clip(reward, -1, 1)



    def _check_done(self, observation):
        ttc, ego_speed, _ = observation

        # Collision Detected
        if ttc < 1.0:
            print("Collision")
            return True
        
        # Ego stop
        if ego_speed <= 0.1:
            print("Ego stopped")
            return True
        
        
        
        return False