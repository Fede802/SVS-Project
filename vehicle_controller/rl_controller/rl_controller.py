from stable_baselines3 import PPO
import os, carla_utility, carla

class RLController:
    def __init__(self):
        self.rl_velocity = PPO.load(os.path.join(os.path.dirname(__file__), 'model', 'working_cc_model.zip'))
        self.rl_distance = PPO.load(os.path.join(os.path.dirname(__file__), 'model', 'working_braking_model.zip'))

    def apply_control(self, control_info, current_velocity, min_depth):
        min_permitted_distance = carla_utility.compute_security_distance(current_velocity) + control_info.min_permitted_offset + 7
        distance_error = min_depth - min_permitted_distance
        if distance_error > 0:
            action, _ = self.rl_velocity.predict([control_info.target_velocity, current_velocity], deterministic=True)
        else:
            action, _ = self.rl_distance.predict([min_permitted_distance, min_depth], deterministic=True)
        action = action[0]
        throttle = action if action >= 0 else 0
        brake = -action if action < 0 else 0
        return carla.VehicleControl(throttle = throttle, brake = brake)