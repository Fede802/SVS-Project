import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla
import debug_utility


class CarlaEnv(gym.Env):
    
    TARGET_SPEED = 16.6667 #60 km/h
    MAX_RELATIVE_SPEED = 100

    def __init__(self):
        super(CarlaEnv, self).__init__()

        #Set up the CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.load_world('Town04')

        #Define the action for ego vehicle [throttle, break]
        self.action_space = spaces.Box(
            low=np.array([0.0, 0.0]), # [min(throttle), min(break)]
            high=np.array([1.0, 1.0]), # [max(throttle), max(break)]
            dtype=np.float32
        )

        #Define the observation space
        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, -30.0]), # [min(distance), min(ego_speed), min(relative_speed)] in m/s
            high=np.array([250, 30.0, 30.0]), # [max(distance, max(ego_speed), max(relative_speed)] in m/s
            dtype=np.float32
        )

        #Configure the default veicle
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        self.spawn_points = self.world.get_map().get_spawn_points()

        self.ego_vehicle = None
        self.leader_vehicle = None
        self.radar_sensor = None

        self.radar_data = {  # Default radar data
            "distance": np.nan,  # Default distance if no object is detected
            "relative_speed": np.nan  # Default relative speed
        }

        #For oscillation
        self.previous_action = 0.0

    def spawn_vehicles(self):
        spawn_point_leader = self.spawn_points[1]
        self.leader_vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point_leader)

        # Spawn point ego vehicle at random position
        safe_distance = (self.TARGET_SPEED / 10)**2 # To avoid collisions
        random_distance = rnd.uniform(safe_distance + 5, 100.0)

        ego_vehicle = carla.Location(
            x = spawn_point_leader.location.x - random_distance,
            y = spawn_point_leader.location.y,
            z = spawn_point_leader.location.z
        )

        print("Ego spawned at")
        print(f"x: {ego_vehicle.x}, y: {ego_vehicle.y}, z: {ego_vehicle.z}")

        ego_rotation = spawn_point_leader.rotation

        self.ego_vehicle = self.world.spawn_actor(
            self.vehicle_bp, 
            carla.Transform(ego_vehicle, ego_rotation))

    
    def reset(self, seed=42):
        print("----RESET----")
        # Destroy existing actors
        if self.leader_vehicle is not None:
            self.leader_vehicle.destroy()
        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()
        if self.radar_sensor is not None:
            self.radar_sensor.destroy()
            self.radar_sensor = None 
        
        # Spawn vehicles
        self.spawn_vehicles()

        #Add radar sensor
        radar_bp = self.blueprint_library.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '2')
        radar_bp.set_attribute('vertical_fov', '2')
        radar_bp.set_attribute('range', '250')  # Maximum range of the radar

        self.radar_sensor = self.world.spawn_actor(
            radar_bp,
            carla.Transform(carla.Location(x=2.5, z=1.0)),
            attach_to=self.ego_vehicle
        )

        # Get data from radar sensor
        self.radar_sensor.listen(self._radar_callback)

        # Reset veichle state
        random_velocity = rnd.uniform(2.7778, self.TARGET_SPEED + 1) # ~ min: 10km/h max: 60km/h
        self.ego_vehicle.set_target_velocity(carla.Vector3D(random_velocity, 0, 0))
        self.leader_vehicle.set_target_velocity(carla.Vector3D(random_velocity, 0, 0))
        
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
            print(f"Detected: {self.radar_data['distance']} m, Relative Speed: {self.radar_data['relative_speed']} m/s")
        else:
            self.radar_data["distance"] = np.nan
            self.radar_data["relative_speed"] = np.nan
            print(f"Detected: {self.radar_data['distance']} m, Relative Speed: {self.radar_data['relative_speed']} m/s")
        
    def step(self, action):

        throttle = float(max(0.0, action[0]))
        brake = float(max(0.0, action[1]))

        # Apply action to throttle and brake
        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        self.ego_vehicle.apply_control(control)

        self.world.tick()
                
        observation = self._get_observation()
        reward = self._compute_reward(observation, action)
        done = self._check_done(observation)

        return observation, reward, done, False, {}
    
    def _get_observation(self):
        
        if self.radar_data["distance"] != np.nan:
            distance = abs(self.radar_data["distance"])
        else:
            distance = np.nan
        
        if self.radar_data["relative_speed"] != np.nan:
            relative_speed = self.radar_data["relative_speed"]
        else:
            relative_speed = np.nan
        

        ego_speed = np.linalg.norm([
            self.ego_vehicle.get_velocity().x,
            self.ego_vehicle.get_velocity().y,
            self.ego_vehicle.get_velocity().z
        ])

        print(f"Current ego speed: {ego_speed} m/s, Distance: {distance} m, Relative Speed: {relative_speed} m/s")
        return np.array([distance, abs(ego_speed), relative_speed], dtype=np.float32)

    def _compute_reward(self, observation, action):
        distance, ego_speed, relative_speed = observation # Leader speed is relative to ego vehicle

        tolerance = 1.0
        absolute_speed = ego_speed + relative_speed
        collision_time = distance / max(relative_speed, 0.1) # No zero velocity
        no_detected_object = relative_speed == self.MAX_RELATIVE_SPEED
        mantain_target_speed = abs(self.TARGET_SPEED - tolerance) <= ego_speed <= abs(self.TARGET_SPEED + tolerance)
        mantain_relative_speed = abs(absolute_speed) < ego_speed < abs(absolute_speed)
        large_collision_time = collision_time > 5.0
        good_collision_time = 2.4 <= collision_time <= 4.0
        bad_collision_time = collision_time < 1.0
        have_strong_breake = abs(self.previous_action - action[1]) > 0.80
        have_strong_throttle = abs(self.previous_action - action[0]) > 0.70

        # Reward
        reward = 0.0

        #if no object detected mantain TARGET SPEED
        if no_detected_object:
            if mantain_target_speed: 
                reward += 1
            else: reward -= 100 * abs(self.TARGET_SPEED - ego_speed)
        else:
            if large_collision_time:
                reward += 1
            else:
                #if object is detected and 2.4 <= collision time <= 5.0  and absolute speed - 1 < ego speed < absolute speed - 1
                if good_collision_time:
                    if mantain_relative_speed:
                        reward += 5 + abs(ego_speed)
                    else: 
                        reward += -10 + abs(relative_speed)
                #if object is detected and but collision time < 1.0
                if bad_collision_time:
                    reward += -100
        
        # Penality for oscillation
        if have_strong_throttle:
            reward -= 1 + abs(action[0])
        else:
            reward += 1
        
        if have_strong_breake:
            reward -= 2 + abs(action[1])
        else:
            reward += 1
        
        return max(reward, -200) # Avoid large negative rewards



    def _check_done(self, observation):
        distance, ego_speed, relative_speed = observation
        collision_time = distance / max(relative_speed, 1) # No zero velocity
        ego_stopped = abs(ego_speed) < 0.1

        if ego_stopped:
            print("Ego vehicle has stopped.")
            return True
        
        if collision_time < 1.0 and relative_speed != self.MAX_RELATIVE_SPEED:
            print("Collision time detected.")
            return True
        
        return False