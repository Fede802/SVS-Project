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
        
        # Observation space: [distance, leader_speed, ego_speed, relative_speed]
        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, 0.0, -1.0]),
            high=np.array([100.0, 30.0, 30.0, 1.0]),
            dtype=np.float32
        )

        # Action space: Continuous acceleration between -1.0 (max brake) and 1.0 (max throttle)
        self.action_space = spaces.Box(
            low=np.array([-1.0]),
            high=np.array([1.0]),
            dtype=np.float32
        )
        
        # Configura il veicolo
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        self.spawn_points = self.world.get_map().get_spawn_points()

        self.leader_vehicle = None
        self.ego_vehicle = None
        self.radar_sensor = None


        self.radar_data = {  # Default radar data
            "distance": 100.0,  # Default distance if no object is detected
            "relative_speed": 0.0  # Default relative speed
        }

        #Track oscillations in actions
        self.previous_action = 0.0

    def spawn_vehicles(self):
        # Spawn vehicles
        spawn_point_ego = self.spawn_points[1]
        self.ego_vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point_ego)

        # Leader vehicle spawn at random distance
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
            carla.Transform(leader_location, leader_rotation)
        )

    
    def reset(self, seed=None):
        # Destroy existing actors
        if self.leader_vehicle is not None:
            self.leader_vehicle.destroy()
        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()
        if self.radar_sensor is not None:
            self.radar_sensor.destroy()
            self.radar_sensor = None

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

        # Wait for simulation to progress
        self.world.tick()

        
        # Return initial observation
        return self._get_observation(), {}
    
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

        # Convert the numpy action to a native float for CARLA
        throttle = float(max(0.0, action[0]))
        brake = float(max(0.0, -action[0]))

        print(f"Throttle: {throttle}, Brake: {brake}")

        #Apply action to throttle and brake
        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        self.leader_vehicle.apply_control(control)

        self.world.tick()

        observation = self._get_observation()
        reward = self._compute_reward(observation, action)
        done = self._check_done(observation)

        return observation, reward, done, False, {}
    
    def _get_observation(self):
        distance = self.radar_data["distance"]
        relative_speed = self.radar_data["relative_speed"]

        leader_speed = np.linalg.norm([
            self.leader_vehicle.get_velocity().x,
            self.leader_vehicle.get_velocity().y,
            self.leader_vehicle.get_velocity().z
        ])

        ego_speed = np.linalg.norm([
            self.ego_vehicle.get_velocity().x,
            self.ego_vehicle.get_velocity().y,
            self.ego_vehicle.get_velocity().z
        ])

        return np.array([distance, leader_speed, ego_speed, relative_speed], dtype=np.float32)

    def _compute_reward(self, observation, action):
        distance, leader_speed, ego_speed, relative_speed = observation

        if 10.0 < distance < 30.0: 
            distance_reward = 1.0
        elif distance >= 30.0: 
            distance_reward = -2.0
        elif distance <= 5.0: 
            distance_reward = -10.0
        else:
            distance_reward = -1.0 


        leader_stopped = leader_speed < 0.1
        if leader_stopped and distance <= 5.0:
            distance_reward -= 5.0
        elif leader_stopped and distance >= 50.0:
            distance_reward -= 5.0
        elif leader_stopped and 10.0 < distance < 20.0:
            distance_reward += 10.0
        
        # Speed reward
        speed_reward = -abs(leader_speed - self.TARGET_SPEED) / self.TARGET_SPEED

        # Action stability penalty (6)
        action_penalty = -abs(action[0] - self.previous_action)

        self.previous_action = action[0]

        return distance_reward + speed_reward + action_penalty

    def _check_done(self, observation):
        distance, leader_speed, ego_speed, relative_speed = observation

        leader_stopped = leader_speed < 0.1

        # Stop the simulation if the leader vehicle is too longer than 50 meters from the ego vehicle
        if distance >= 50.0 and leader_stopped:
            print("Leader vehicle is too longer than 50 meters from the ego vehicle.")
            return True
        
        if distance <= 5.0:
            print("Collision detected.")
            return True
        
        return False