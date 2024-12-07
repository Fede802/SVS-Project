import numpy as np
from collections import deque
import time

class PIDController:
   # def __init__(self, kp_speed = 1.5, ki_speed = 0.5, kd_speed = 0.001, dt_speed = 0.03, 
    #             kp_distance = 0.8, ki_distance = 0.5, kd_distance = 0.001, dt_distance = 0.03):
        # PID parameters for speed
    def __init__(self, kp_speed = 0.4, ki_speed = 0.2, kd_speed = 0.01, dt_speed = 0.05, 
                kp_distance = 1.6, ki_distance = 0.01, kd_distance = 0.7, dt_distance = 0.05):
        self.K_P_speed = kp_speed
        self.K_I_speed = ki_speed
        self.K_D_speed = kd_speed
        self.dt_speed = dt_speed
        self.e_buffer_speed = deque(maxlen=30)
        self.last_update_speed_time = time.time()
        # PID parameters for distance
        self.K_P_distance = kp_distance
        self.K_I_distance = ki_distance
        self.K_D_distance = kd_distance
        self.dt_distance = dt_distance
        self.e_buffer_distance = deque(maxlen=30)
        self.last_update_distance_time = time.time()

    def compute_pid_control_speed(self, target_speed, current_speed):
        """
        Compute the throttle control based on PID equations for speed.
        
        :param target_speed: Target speed in Km/h
        :param current_speed: Current speed of the vehicle in Km/h
        :return: Throttle control in the range [0, 1]
        """
        target_speed = target_speed / 3.6
        current_speed = current_speed / 3.6
        e = target_speed - current_speed
        self.e_buffer_speed.append(e)
        self.dt_speed = time.time() - self.last_update_speed_time
        self.last_update_speed_time = time.time()
        if len(self.e_buffer_speed) >= 2:
            de = (self.e_buffer_speed[-1] - self.e_buffer_speed[-2]) / self.dt_speed
            ie = sum(self.e_buffer_speed) * self.dt_speed
        else:
            de = 0.0
            ie = 0.0

        return np.clip(
            (self.K_P_speed * e) + (self.K_D_speed * de) + (self.K_I_speed * ie),
            0.0,
            1.0
        )

    def compute_pid_control_distance(self, min_permitted_distance, current_distance):
        """
        Compute the brake control based on PID equations for distance.
        
        :param min_permitted_distance: Minimum permitted distance from the vehicle in front
        :param current_distance: Current distance from the vehicle in front
        :return: Brake control in the range [0, 1]
        """
        e = min_permitted_distance - current_distance
        self.e_buffer_distance.append(e)
        # print(current_distance)
        self.dt_distance = time.time() - self.last_update_distance_time
        self.last_update_distance_time = time.time()
        if len(self.e_buffer_distance) >= 2:
            de = (self.e_buffer_distance[-1] - self.e_buffer_distance[-2]) / self.dt_distance
            ie = sum(self.e_buffer_distance) * self.dt_distance
        else:
            de = 0.0
            ie = 0.0

        return np.clip(
            (self.K_P_distance * e) + (self.K_D_distance * de) + (self.K_I_distance * ie),
            0.0,
            1.0
        ) 
    
#old compute control idea
# errore_vel = velocita_target - (velocita_corrente * 3.6)
# errore_vel_perc = abs(errore_vel) * 100 / velocita_target

# if errore_vel > 0:
#             throttle = 1.0 * errore_vel_perc
#         else:
#             brake = 1.0 * errore_vel_perc    
