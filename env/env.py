import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla
import debug_utility


class AccEnv(gym.Env):
    
    TARGET_SPEED = 16.6667 #60 km/h
    SPAWN_POINT = carla.Location(x=2393, y=6000, z=167)

    def __init__(self):
        super(AccEnv, self).__init__()

        #Set up the CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(100.0)
        self.world = self.client.get_world()
        self.spectator = self.world.get_spectator()
        self.spectator.set_transform(carla.Transform(self.SPAWN_POINT, carla.Rotation(yaw = -88.2)))

        #Define the action for ego vehicle [throttle, break]
        self.action_space = spaces.Box(
            low=np.array([0.0, 0.0]), # [min(throttle), min(break)]
            high=np.array([1.0, 1.0]), # [max(throttle), max(break)]
            dtype=np.float64
        )

        #Define the observation space
        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, -30.0, self.TARGET_SPEED, 0]), # [min(distance), min(ego_speed), min(relative_speed), target_speed, object detected] in m/s
            high=np.array([50, 30.0, 30.0, self.TARGET_SPEED, 1]), # [max(distance, max(ego_speed), max(relative_speed), target_speed, object detected] in m/s
            dtype=np.float64
        )

        #Configure the default vehicle 
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        self.spawn_points = self.world.get_map().get_spawn_points()

        self.ego_vehicle = None
        self.leader_vehicle = None
        self.radar_sensor = None

        self.radar_data = {  # Default radar data
            "distance": 50,  # Default distance if no object is detected
            "relative_speed": 0.0,  # Default relative speed
            "object_detected": 0
        }


    def spawn_vehicles(self):
        spawn_point_leader = carla.Transform(self.SPAWN_POINT, carla.Rotation(yaw = -88.2))
        self.leader_vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point_leader)

        # Spawn point ego vehicle at random position
        random_distance = rnd.uniform(50.00, 100.0)

        ego_vehicle = carla.Location(
            x = self.SPAWN_POINT.x - 1,
            y = self.SPAWN_POINT.y + 50.0,
            z = self.SPAWN_POINT.z
        )

        #print("Ego spawned at")
        #print(f"x: {ego_vehicle.x}, y: {ego_vehicle.y}, z: {ego_vehicle.z}")

        ego_rotation = spawn_point_leader.rotation

        self.ego_vehicle = self.world.spawn_actor(
            self.vehicle_bp, 
            carla.Transform(ego_vehicle, ego_rotation))

    
    def reset(self, seed=42):
        #print("----RESET----")
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
        radar_bp.set_attribute('range', '50')  # Maximum range of the radar

        self.radar_sensor = self.world.spawn_actor(
            radar_bp,
            carla.Transform(carla.Location(x=2.5, z=1.0)),
            attach_to=self.ego_vehicle
        )

        # Get data from radar sensor
        self.radar_sensor.listen(self._radar_callback)

        # Reset vehicle state
        random_ego_velocity = rnd.uniform(9.0, self.TARGET_SPEED) # ~ min: 32,4km/h max: 60km/h
        random_leader_velocity = rnd.uniform(9.0, self.TARGET_SPEED + 5) # ~ min: 32,4km/h max: 60km/h
        self.ego_vehicle.set_target_velocity(carla.Vector3D(0, -random_ego_velocity, 0))
        self.leader_vehicle.set_target_velocity(carla.Vector3D(0, -random_leader_velocity, 0))
        
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

        throttle = float(max(0.0, action[0] - action[1]))
        brake = float(max(0.0, action[1] - action[0]))

        # Apply action to throttle and brake
        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        self.ego_vehicle.apply_control(control)

        self.world.tick()
                
        observation = self._get_observation()
        reward = self._compute_reward(observation)
        done = self._check_done(observation)

        print(f"Reward: {reward}")
        

        return observation, reward, done, False, {}
    
    def _get_observation(self):
        
        distance = abs(self.radar_data["distance"])
        relative_speed = self.radar_data["relative_speed"]
        object_detected = self.radar_data["object_detected"]
        

        ego_speed = np.linalg.norm([
            self.ego_vehicle.get_velocity().x,
            self.ego_vehicle.get_velocity().y,
            self.ego_vehicle.get_velocity().z
        ])

        return np.array([distance, abs(ego_speed), relative_speed, self.TARGET_SPEED, object_detected], dtype=np.float32)

    def _compute_reward(self, observation):
        distance, ego_speed, relative_speed, target_speed, object_detected = observation # Leader speed is relative to ego vehicle
        
        security_distance = (ego_speed / 10) ** 2
        absolute_speed = abs(relative_speed) + abs(ego_speed)
        reward = rnd.uniform(0.1, 0.5)

        #No Object detected
        if object_detected == 0:
            
            #Mantain target speed
            if target_speed - 2 <= ego_speed <= target_speed + 2:
                reward += 1
            else:
                reward -= 1 * abs(target_speed - ego_speed)
        
        else:
            #Object Detected
            #Mantain security distance
            if security_distance - 1 <= distance <= security_distance + 1:
                reward += 1
            else:
                reward -= 1 * abs(security_distance - distance)
            
            # Leader velocity > target velocity
            if target_speed <= absolute_speed:
                #Mantain target speed
                if target_speed - 2 <= ego_speed <= target_speed + 2:
                    reward += 1
                else:
                    reward -= 1 * abs(target_speed - ego_speed)
            else:
                #Maintain leader velocity
                if absolute_speed - 1 <= ego_speed <= absolute_speed + 1:
                    reward += 1
                else:
                    reward -= 1 * abs(absolute_speed - ego_speed)
                    
            #Collision
            if distance < 1:
                reward -= 1
            
        return max(-10, reward)



    def _check_done(self, observation):
        distance, ego_speed, _, _, object_detected = observation

        # Collision Detected
        if object_detected == 1 and distance <= 1:
            print("Collision")
            return True
        
        # Ego stop
        if ego_speed <= 0.1:
            print("Ego stopped")
            return True
        
        
        
        return False