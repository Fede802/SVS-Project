import numpy as np
from collections import deque

class PIDController:
    def __init__(self, kp_speed, ki_speed, kd_speed, dt_speed, kp_distance, ki_distance, kd_distance, dt_distance):
        # PID parameters for speed
        self.K_P_speed = kp_speed
        self.K_I_speed = ki_speed
        self.K_D_speed = kd_speed
        self.dt_speed = dt_speed
        self.e_buffer_speed = deque(maxlen=30)
        
        # PID parameters for distance
        self.K_P_distance = kp_distance
        self.K_I_distance = ki_distance
        self.K_D_distance = kd_distance
        self.dt_distance = dt_distance
        self.e_buffer_distance = deque(maxlen=30)

    def compute_pid_control_speed(self, target_speed, current_speed):
        """
        Compute the throttle control based on PID equations for speed.
        
        :param target_speed: Target speed in Km/h
        :param current_speed: Current speed of the vehicle in Km/h
        :return: Throttle control in the range [0, 1]
        """
        e = target_speed - current_speed
        self.e_buffer_speed.append(e)

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
