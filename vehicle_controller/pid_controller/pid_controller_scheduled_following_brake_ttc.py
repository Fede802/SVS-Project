import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
import numpy as np
from collections import deque
import carla_utility, time, carla

class PID:
    def __init__(self, dt, buffer_size, kp=0.2, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.e_buffer = deque(maxlen=buffer_size) if buffer_size != None and buffer_size > 0 else deque()

    def resetBuffer(self):
        self.e_buffer.clear()

    def compute_control(self, target, current):
        e = target - current
        self.e_buffer.append(e)
        ie = np.sum(self.e_buffer) * self.dt
        de = (self.e_buffer[-1] - self.e_buffer[-2]) / self.dt if len(self.e_buffer) > 1 else 0.0

        return np.clip((self.kp * e) + (self.kd * de) + (self.ki * ie), 0.0, 1.0)
    
class PIDController:
    def __init__(self, tc, buffer_size = 42, kp_velocity = 0.4, ki_velocity = 0.2, kd_velocity = 0.01, 
                kp_distance = 0.3, ki_distance = .9, kd_distance = 0.0):
        self.pid_velocity = PID(tc, buffer_size, kp_velocity, ki_velocity, kd_velocity)
        self.pid_distance = PID(tc, buffer_size, kp_distance, ki_distance, kd_distance)
        self.update_frequency = tc
        self.last_update = 0
        self.last_throttle = 0
        self.last_brake = 0
        
    def apply_control(self, vehicle: carla_utility.VehicleWithRadar):
        if vehicle.acc_info.is_active():
            if vehicle.acc_info.reset:
                self.last_update = 0
                vehicle.acc_info.reset = False
                
            if(time.time() - self.last_update > self.update_frequency):
                min_permitted_distance = carla_utility.compute_security_distance(vehicle.vehicle.get_velocity().length() * 3.6) + vehicle.acc_info.min_permitted_offset
                distance_error = vehicle.min_depth - min_permitted_distance
                self.last_throttle = 0
                self.last_brake = 0
                print("Distance Error: ", vehicle.min_depth, min_permitted_distance,distance_error)
                if distance_error > 30:
                    print("Throttle1")
                    control =  carla.VehicleControl(throttle = self.pid_velocity.compute_control(vehicle.acc_info.target_velocity, vehicle.vehicle.get_velocity().length() * 3.6))
                    self.pid_distance.resetBuffer()
                elif distance_error > 0:
                    print("Throttle2")
                    ego_velocity = vehicle.vehicle.get_velocity().length() * 3.6
                    relative_velocity = vehicle.relative_velocity * 3.6
                    control =  carla.VehicleControl(throttle = self.pid_velocity.compute_control(ego_velocity + relative_velocity, ego_velocity))   
                elif vehicle.min_depth > vehicle.acc_info.min_permitted_offset:
                    brake = self.pid_distance.compute_control(0, 1/vehicle.min_ttc)
                    print("Brake: ", brake)
                    control = carla.VehicleControl(brake = brake)
                    # self.pid_velocity.resetBuffer()
                else:
                    control = carla.VehicleControl(brake = 0.6)
                    # self.pid_distance.resetBuffer()
                self.last_throttle = control.throttle    
                self.last_brake = control.brake    
                self.last_update = time.time()
                vehicle.vehicle_control = control
            else:
                
                vehicle.vehicle_control = carla.VehicleControl(throttle = self.last_throttle, brake = self.last_brake)
        else:
            
            vehicle.vehicle_control = carla.VehicleControl()    