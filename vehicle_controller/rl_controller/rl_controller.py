from stable_baselines3 import PPO
import os, carla_utility, carla

class RLController:
    def __init__(self):
        self.rl_velocity = PPO.load(os.path.join(os.path.dirname(__file__), 'model', 'working_cc_model.zip'))
        self.rl_distance = PPO.load(os.path.join(os.path.dirname(__file__), 'model', 'working_braking_model.zip'))
        self.braking = False

    def apply_control(self, vehicle):
        if vehicle.acc_info.is_active():
            if vehicle.acc_info.reset:
                vehicle.acc_info.reset = False
                self.braking = False
            
            min_permitted_distance = carla_utility.compute_security_distance(vehicle.vehicle.get_velocity().length() * 3.6) + vehicle.acc_info.min_permitted_offset + 7
            distance_error = vehicle.min_depth - min_permitted_distance
            if self.braking and distance_error > 50:
                self.braking = False
            elif not self.braking and distance_error < 10:
                self.braking = True    

            if not self.braking > 0:
                action, _ = self.rl_velocity.predict([vehicle.acc_info.target_velocity, vehicle.vehicle.get_velocity().length() * 3.6], deterministic=True)
            else:
                action, _ = self.rl_distance.predict([min_permitted_distance, vehicle.min_depth], deterministic=True)
            action = action[0]
            throttle = action if action >= 0 else 0
            brake = -action if action < 0 else 0
            vehicle.vehicle_control = carla.VehicleControl(throttle = throttle, brake = brake)
        else:
            vehicle.vehicle_control = carla.VehicleControl()