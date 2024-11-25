import random as rnd
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla

#Template for enviroment
class CarlaEnv(gym.Env):
    
    TARGET_SPEED = 16.6667 #60 km/h

    def __init__(self):
        super(CarlaEnv, self).__init__()
        
        #Initialize carla client and world
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.load_world('Town04')
        
        #Configure space actions and observation
        self.action_space = spaces.Discrete(3) # 0: Decelerating, 1: Keeping speed, 2: Accelerating
        #Configure space observation
        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, -1.0]), # [distance, speed, relative speed]
            high=np.array([100.0, 30.0, 1.0]), #[max_distance, max_speed]
            dtype=np.float32
        )
        
        # Configura il veicolo
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        self.spawn_points = self.world.get_map().get_spawn_points()

        self.leader_vehicle = None
        self.ego_vehicle = None

        #Simulation state
        self.radar_sensor = None

        self.radar_data = {  # Default radar data
            "distance": 100.0,  # Default distance if no object is detected
            "relative_speed": 0.0  # Default relative speed
        }

    def spawn_vehicles(self):
        # Spawn vehicles
        spawn_point_ego = self.spawn_points[1]
        self.ego_vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point_ego)

        #Spawn at different random distance
        random_distance = rnd.uniform(20.0, 100.0)
        # Calculate the leader's location 10 meters behind the ego vehicle
        leader_location = carla.Location(
            x=spawn_point_ego.location.x - random_distance,
            y=spawn_point_ego.location.y,
            z=spawn_point_ego.location.z,
        )
        leader_rotation = spawn_point_ego.rotation  # Keep the same rotation as the ego

        self.leader_vehicle = self.world.spawn_actor(
            self.vehicle_bp,
            carla.Transform(carla.Location(leader_location), leader_rotation),
            attach_to=self.ego_vehicle
        )

    
    def reset(self):
        # Destroy existing actors
        if self.leader_vehicle is not None:
            self.leader_vehicle.destroy()
        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()

        self.spawn_vehicles()
        
        radar_bp = self.blueprint_library.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '30')
        radar_bp.set_attribute('vertical_fov', '10')
        radar_bp.set_attribute('range', '50')  # Maximum range of the radar

        self.radar_sensor = self.world.spawn_actor(
            radar_bp,
            carla.Transform(carla.Location(x=2.5, z=1.0)),
            attach_to=self.leader_vehicle
        )
        self.radar_sensor.listen(self._radar_callback)


        # Reset veichle state
        self.leader_vehicle.set_target_velocity(carla.Vector3D(self.TARGET_SPEED, 0, 0))
        self.ego_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        
        # Return initial observation
        return self._get_observation()
    
    def _radar_callback(self, radar_data):
        distances = []
        velocities = []

        for detection in radar_data:
            distances.append(detection.depth) #Distance to detect object
            velocities.append(detection.velocity) #Velocity of detect object

        # Use the closest detected object as the relevant one for control
        if distances:
            self.radar_data = {
                "distance": min(distances),
                "relative_speed": velocities[np.argmin(distances)]
            }
        else:
            # Default if no object detected
            self.radar_data = {
                "distance": 100.0,  # Assume max distance
                "relative_speed": 0.0
            }

    def step(self, action):
        # Apply action to the leader vehicle
        if action == 0:  # Decelerate
            self.leader_vehicle.set_target_velocity(carla.Vector3D(self.TARGET_SPEED * 0.5, 0, 0))
        elif action == 1:  # Maintain speed
            self.leader_vehicle.set_target_velocity(carla.Vector3D(self.TARGET_SPEED, 0, 0))
        elif action == 2:  # Accelerate
            self.leader_vehicle.set_target_velocity(carla.Vector3D(self.TARGET_SPEED * 1.2, 0, 0))
        
        # Wait for simulation to progress
        self.world.tick()

        # Gather observation, compute reward, and check if the simulation is done
        observation = self._get_observation()
        reward = self._compute_reward(observation)
        done = self._check_done()
        
        return observation, reward, done, {}
    
    def _get_observation(self):
        
        
        # Default observation if radar hasn't yet detected anything
        if self.radar_data is None:
            return np.array([100.0, self.TARGET_SPEED, 0.0], dtype=np.float32)
        
        # Extract distance and relative speed from radar data
        distance = self.radar_data["distance"]
        relative_speed = self.radar_data["relative_speed"]

        # Leader speed from its velocity
        leader_speed = np.linalg.norm([
            self.leader_vehicle.get_velocity().x,
            self.leader_vehicle.get_velocity().y,
            self.leader_vehicle.get_velocity().z
        ])
        
        return np.array([distance, leader_speed, relative_speed], dtype=np.float32)

    def _compute_reward(self, observation):
        distance, leader_speed, relative_speed = observation

        #Mantaining safe distance and target speed reward
        if 10.0 < distance < 30.0: distance_reward = 1.0
        else: distance_reward = -1.0

        speed_reward = -abs(leader_speed - self.TARGET_SPEED)

        return distance_reward + speed_reward

    def _check_done(self):

        # Check if the leader vehicle stops too close to the ego vehicle
        if self.radar_data and self.radar_data["distance"] < 5.0:  # Unsafe close distance
            return True
        
        return False