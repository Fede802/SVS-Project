import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
import numpy as np
from collections import deque
import carla_utility, time, carla

class PID:
    def __init__(self, buffer_size, kp=0.2, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = -1
        self.e_buffer = deque(maxlen=buffer_size) if buffer_size != None and buffer_size > 0 else deque()

    def resetBuffer(self):
        self.e_buffer.clear()
        self.dt = -1

    def compute_control(self, target, current):
        e = target - current
        self.e_buffer.append(e)
        if self.dt == -1:
            self.dt = time.time()
            ie = 0.0
            de = 0.0
        else:
            self.dt = time.time() - self.dt    
            ie = np.sum(self.e_buffer) * self.dt
            de = (self.e_buffer[-1] - self.e_buffer[-2]) / self.dt #don't needed -> if len(self.e_buffer
        
        return np.clip((self.kp * e) + (self.kd * de) + (self.ki * ie), 0.0, 1.0)
    
class PIDController:
    def __init__(self, buffer_size = 42, kp_velocity = 0.4, ki_velocity = 0.2, kd_velocity = 0.01, 
                kp_distance = 0.03, ki_distance = 0.9, kd_distance = 0.0):
        self.pid_velocity = PID(buffer_size, kp_velocity, ki_velocity, kd_velocity)
        self.pid_distance = PID(buffer_size, kp_distance, ki_distance, kd_distance)
        self.following = False

        

    def apply_control(self, vehicle: carla_utility.VehicleWithRadar):
        if vehicle.acc_info.is_active():
            if vehicle.acc_info.reset:
                vehicle.acc_info.reset = False
                self.following = False
            min_permitted_distance = carla_utility.compute_security_distance(vehicle.vehicle.get_velocity().length() * 3.6) + vehicle.acc_info.min_permitted_offset
            distance_error = vehicle.min_depth - min_permitted_distance
            
            if self.following and distance_error > 50:
                    self.following = False
            elif not self.following and distance_error < 10:
                    self.following = True    

            if not self.following:
                control =  carla.VehicleControl(throttle = self.pid_velocity.compute_control(vehicle.acc_info.target_velocity, vehicle.vehicle.get_velocity().length() * 3.6))
            elif self.following and distance_error > 0:
                ego_velocity = vehicle.vehicle.get_velocity().length() * 3.6
                relative_velocity = vehicle.relative_velocity * 3.6
                control =  carla.VehicleControl(throttle = self.pid_velocity.compute_control(ego_velocity + relative_velocity, ego_velocity)) 
                self.pid_distance.resetBuffer()   
            elif vehicle.min_depth > vehicle.acc_info.min_permitted_offset:
                brake = self.pid_distance.compute_control(0, 1/vehicle.min_ttc)
                control = carla.VehicleControl(brake = brake)
                self.pid_velocity.resetBuffer()
            else:
                control = carla.VehicleControl(brake = 0.6)
                self.pid_distance.resetBuffer()
            vehicle.vehicle_control = control
        else:
            vehicle.vehicle_control = carla.VehicleControl()    