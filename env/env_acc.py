import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla
import debug_utility
import carla_utility


class AccEnv(gym.Env):
    
    MAX_TARGET_VELOCITY = 130
    TARGET_VELOCITY = -1 #set randomly in reset or with setter
    RADAR_RANGE_OFFSET = 20
    RADAR_RANGE = carla_utility.compute_security_distance(MAX_TARGET_VELOCITY) + RADAR_RANGE_OFFSET
    SPAWN_POINT = carla.Transform(carla.Location(x=2388, y=6164, z=179), carla.Rotation(yaw = -88.2))

    def __init__(self, seed):
        super(AccEnv, self).__init__()
        rnd.seed(seed)
        #Set up the CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(100.0)
        self.world = self.client.get_world()
    
        #Define the action for ego vehicle
        self.action_space = spaces.Box(
            low=-1.0, #max brake
            high=1.0, #max thottle
            dtype=float
        )
        
        #Define the observation space velocity in Km/h, distance in meters
        self.observation_space = spaces.Box(
            low=np.array([30.0, 0.0, 0.0, 7.0, 0.0]), # [target velocity, current velocity, detected, security distance, current distance] in m/s
            high=np.array([150.0, 300.0, 1.0, carla_utility.compute_security_distance(180) + 21, self.RADAR_RANGE]), # [target velocity, current velocity, detected, security distance, current distance] in m/s
            dtype=np.float32
        )

        self.min_distance_setting = [7, 14, 21]
        self.ego_vehicle = None
        self.leader_vehicle = None
        self.radar_sensor = None
        self.radar_data = {}
        self.last_observation = None
        self.step_count = 0

    def setTargetSpeed(self, speed):
        self.TARGET_VELOCITY = speed

    def setMinDistanceOffset(self, offset):
        self.MIN_DISTANCE_OFFSET = offset    

    def spawn_vehicles(self, random_ego_velocity):
        
        ego_offset = rnd.randint(0, 50)
        ego_security_distance = carla_utility.compute_security_distance(random_ego_velocity) + self.MIN_DISTANCE_OFFSET
        security_distance_spawn_offset = 150
        other_vehicle_offset = rnd.randint(ego_offset + ego_security_distance, ego_offset + ego_security_distance + security_distance_spawn_offset)

        self.ego_vehicle = carla_utility.spawn_vehicle_bp_at(self.world, 'vehicle.tesla.cybertruck', spawn_point=carla.Transform(debug_utility.get_point_from_trasform(self.SPAWN_POINT, ego_offset), self.SPAWN_POINT.rotation))
        self.leader_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(self.world, self.ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=other_vehicle_offset)


    def reset(self, seed = 42):
        carla_utility.destroy_all_vehicle_and_sensors(self.world) #to avoid spawning bugs
        self.TARGET_VELOCITY = rnd.randint(30, 150)
        random_ego_velocity = rnd.randint(0, 180)
        self.MIN_DISTANCE_OFFSET = self.min_distance_setting[rnd.randint(0, 2)]
        
        # Spawn vehicles and sensor
        self.spawn_vehicles(random_ego_velocity)
        self.radar_sensor = carla_utility.spawn_radar(self.world, self.ego_vehicle, range=self.RADAR_RANGE)
        self.radar_sensor.listen(self._radar_callback)

        random_ego_velocity = random_ego_velocity / 3.6
        random_leader_velocity = rnd.randint(0, 180) / 3.6
        self.ego_vehicle.set_target_velocity(debug_utility.get_velocity_vector(random_ego_velocity, self.SPAWN_POINT.rotation))
        self.leader_vehicle.set_target_velocity(debug_utility.get_velocity_vector(random_leader_velocity, self.SPAWN_POINT.rotation))
        
        self.world.tick() #totaly useless

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
            self.radar_data["current_distance"] = min(distances)
        else:
            self.radar_data["current_distance"] = self.RADAR_RANGE
        
    def step(self, action):
        if action >= 0:
            action = [action, 0]
        else:
            action = [0, -action]    
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=float(action[0]), brake=float(action[1])))
        self.world.tick()
        self.last_observation = self._get_observation()  
        
        
        reward = self._compute_reward(self.last_observation)
        done = self._check_done(self.last_observation)
        print("targetS: ", self.last_observation[0], "currS: ",self.last_observation[1], "det: ", self.last_observation[2], "min_pd: ", self.last_observation[3] ,"min_d: ", self.last_observation[4],"reward:", reward)    
        return self.last_observation, reward, done, False, {}
    
    def _get_observation(self):
        current_velocity = self.ego_vehicle.get_velocity().length() * 3.6
        min_permitted_distance = carla_utility.compute_security_distance(current_velocity) + self.MIN_DISTANCE_OFFSET
        min_depth = self.radar_data["current_distance"]
        distance_error = min_depth - min_permitted_distance
        detected = 0 if distance_error > 0 else 1
        
        return self.TARGET_VELOCITY, current_velocity, detected, min_permitted_distance, min_depth

    def _compute_reward(self, observation):
        if observation[2] == 0:
            return -abs(observation[0] - observation[1])/10# Leader speed is relative to ego vehicle
        else:
            return -abs(observation[3] - observation[4])/10
       
    def _check_done(self, observation):
        self.step_count += 1
        if self.step_count > 1024:
            self.step_count = 0
            carla_utility.destroy_all_vehicle_and_sensors(self.world) #to avoid spawning bugs
            return True
        return False