import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla
import debug_utility
import carla_utility


class AccEnvConstantSpeed(gym.Env):
    # self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float64)
    TARGET_SPEED = 110 #60 km/h
    RADAR_RANGE = 180
    MAX_SECURITY_DISTANCE = carla_utility.compute_security_distance(TARGET_SPEED)
    SPAWN_POINT = carla.Transform(carla.Location(x=2388, y=6164, z=178), carla.Rotation(yaw = -88.2))

    def __init__(self):
        super(AccEnvConstantSpeed, self).__init__()
        rnd.seed(42)
        #Set up the CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(100.0)
        self.world = self.client.get_world()
        self.step_count = 0
        # self.world = self.client.load_world('Town04')

        #Define the action for ego vehicle [throttle, break]
        self.action_space = spaces.Box(
            low=-1.0, # [min(throttle), min(break)]
            high=1.0, # [max(throttle), max(break)]
            dtype=float
        )
        #add security distance?
        #Define the observation space
        self.observation_space = spaces.Box(
            low=np.array([30.0, 0.0]), # [terget, vel, err vel, last err vel] in m/s
            high=np.array([150.0, 300.0]),
            dtype=np.float32
        )

        self.ego_vehicle = None
        self.leader_vehicle = None
        self.radar_sensor = None
        self.last_error = 0
        self.radar_data = {}
        self.last_observation = None
        self.step_count = 0

    def setTargetSpeed(self, speed):
        self.TARGET_SPEED = speed
        # self.MAX_SECURITY_DISTANCE = carla_utility.compute_security_distance(speed)

    def setStartSpeed(self, speed):
        self.START_SPEED = speed    

    def spawn_vehicles(self):
        
        ego_offset = rnd.randint(0, 100)
        # other_vehicle_offset = rnd.randint(ego_offset, 200+ego_offset)

        self.ego_vehicle = carla_utility.spawn_vehicle_bp_at(self.world, 'vehicle.tesla.cybertruck', spawn_point=carla.Transform(debug_utility.get_point_from_trasform(self.SPAWN_POINT, ego_offset), self.SPAWN_POINT.rotation))
        # self.leader_vehicle = carla_utility.spawn_vehicle_bp_in_front_of(self.world, self.ego_vehicle, vehicle_bp_name='vehicle.tesla.cybertruck', offset=10 + other_vehicle_offset)


    def reset(self, seed=42):
        carla_utility.destroy_all_vehicle_and_sensors(self.world) #to avoid spawning bugs

        # Spawn vehicles
        self.spawn_vehicles()
        self.radar_sensor = carla_utility.spawn_radar(self.world, self.ego_vehicle, range=self.RADAR_RANGE)

        # Get data from radar sensor
        self.radar_sensor.listen(self._radar_callback)

        # Reset vehicle state
        # random_ego_velocity = rnd.randint(0, 180) # ~ min: 32,4km/h max: 60km/h

        # self.TARGET_SPEED = rnd.randint(30, 150)
        
        # self.last_error = (self.TARGET_SPEED - (random_ego_velocity))/self.TARGET_SPEED
        # random_ego_velocity = random_ego_velocity / 3.6
        # random_leader_velocity = rnd.randint(0, 16) # ~ min: 32,4km/h max: 60km/h
        self.ego_vehicle.set_target_velocity(debug_utility.get_velocity_vector((self.START_SPEED // 3.6), self.SPAWN_POINT.rotation))
        # self.leader_vehicle.set_target_velocity(debug_utility.get_velocity_vector(random_leader_velocity, self.SPAWN_POINT.rotation))
        
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
            self.radar_data["distance"] = min(distances)
            self.radar_data["relative_speed"] = velocities[np.argmin(distances)]
            self.radar_data["object_detected"] = 1
        else:
            self.radar_data["distance"] = self.RADAR_RANGE
            self.radar_data["relative_speed"] = 0.0
            self.radar_data["object_detected"] = 0
        
    def step(self, action):
        carla_utility.setup_spectator(self.world, self.ego_vehicle.get_transform())
        if action >= 0:
            action = [action, 0]
        else:
            action = [0, -action]    
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=float(action[0]), brake=float(action[1])))
        self.world.tick()
        self.last_observation = self._get_observation()  
        self.last_error = self.last_observation[0]
        # observation = self._get_observation()
        reward = self._compute_reward(self.last_observation)
        done = self._check_done(self.last_observation)
        print("target: ", self.TARGET_SPEED, "currSpeed: ",self.ego_vehicle.get_velocity().length()*3.6, "reward:", reward)    
        return self.last_observation, reward, done, False, {}
    
    def _get_observation(self):
        distance = self.radar_data["distance"]
        relative_speed = self.radar_data["relative_speed"]
        object_detected = self.radar_data["object_detected"]
        ego_speed = self.ego_vehicle.get_velocity().length() * 3.6
        error_speed = (self.TARGET_SPEED - ego_speed)/self.TARGET_SPEED
        last_error = self.last_error
        self.last_error = error_speed
        return self.TARGET_SPEED, ego_speed #np.array([distance, ego_speed, relative_speed, object_detected], dtype=np.float32)

    def _compute_reward(self, observation):
        return -abs(observation[0] - observation[1])/10# Leader speed is relative to ego vehicle
        # if observation[0] < -0.1:
        #     error_reward = -10 * abs(observation[0])
        # elif observation[0] < -0.01:
        #     error_reward = -3 * abs(observation[0])    
        # elif observation[0] < 0.01:
        #     error_reward = -1 * abs(observation[0]) 
        # elif observation[0] < 0.05:
        #     error_reward = -2 * abs(observation[0]) 
        # elif observation[0] < 0.1:
        #     error_reward = -3 * abs(observation[0]) 
        # elif observation[0] < 0.2:
        #     error_reward = -4 * abs(observation[0]) 
        # elif observation[0] < 0.5:
        #     error_reward = -5 * abs(observation[0] )
        # else:
        #     error_reward = -3 * abs(observation[0]) 
        # if (abs(observation[0] )> abs(observation[1])):
        #     error_reward -= 1 * (abs(observation[0]) - abs(observation[1]))

        # return error_reward# Leader speed is relative to ego vehicle
        # distance, ego_speed, relative_speed, object_detected = observation # Leader speed is relative to ego vehicle
        
        # security_distance = (ego_speed // 10) ** 2
        # absolute_speed = abs(relative_speed) + abs(ego_speed)
        # reward = rnd.uniform(0.1, 0.5)

        # #No Object detected
        # if object_detected == 0:
            
        #     #Mantain target speed
        #     if self.TARGET_SPEED - 2 <= ego_speed <= self.TARGET_SPEED + 2:
        #         reward += 1
        #     else:
        #         reward -= 1 * abs(self.TARGET_SPEED - ego_speed)
        
        # else:
        #     #Object Detected
        #     #Mantain security distance
        #     if security_distance - 1 <= distance <= security_distance + 1:
        #         reward += 1
        #     else:
        #         reward -= 1 * abs(security_distance - distance)
            
        #     # Leader velocity > target velocity
        #     if self.TARGET_SPEED <= absolute_speed:
        #         #Mantain target speed
        #         if self.TARGET_SPEED - 2 <= ego_speed <= self.TARGET_SPEED + 2:
        #             reward += 1
        #         else:
        #             reward -= 1 * abs(self.TARGET_SPEED - ego_speed)
        #     else:
        #         #Maintain leader velocity
        #         if absolute_speed - 1 <= ego_speed <= absolute_speed + 1:
        #             reward += 1
        #         else:
        #             reward -= 1 * abs(absolute_speed - ego_speed)
                    
        #     #Collision
        #     if distance < 1:
        #         reward -= 1
            
        # return max(-10, reward)



    def _check_done(self, observation):
        # self.step_count += 1
        # if self.step_count > 1024:
        #     self.step_count = 0
        #     carla_utility.destroy_all_vehicle_and_sensors(self.world) #to avoid spawning bugs
        #     return True
        # # ego_speed, _, _ = observation

        # # Collision Detected
        # if object_detected == 1 and distance <= 2.5:
        #     print("Collision")
        #     return True
        
        # # Ego stop
        # if ego_speed <= 1:
            # print("Ego stopped")
            # return True
        
        
        
        return False