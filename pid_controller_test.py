import numpy as np
from collections import deque
import time

class PIDControllerTest:
    def __init__(self, K_P, K_I, K_D, Tc):
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.Tc = Tc
        self.e_buffer = deque(maxlen=2)
        self.e_buffer.append(0)
        self.e_buffer.append(0) 
        self.last_e = 0
        self.sum_e = 0
        self.last_control = 0

    def compute_control(self, reference, measured):
        e = reference - measured
       
        self.sum_e += e
        de = e - self.last_e
        self.last_e = e
        control = (self.K_P * e) + (self.K_I * self.Tc * self.sum_e) + self.K_D / self.Tc * de
        return np.clip(control, 0.0, 1.0)
        # e = reference - measured
        # kp_coeff = e - self.e_buffer[-1]
        # kd_coeff = e - (2 * self.e_buffer[-1]) + self.e_buffer[-2]
        
        # self.last_control =  np.clip(
        #     (self.last_control+(self.K_P * kp_coeff) + (self.K_D * e * self.Tc) + (self.K_I * kd_coeff / self.Tc)),
        #     0.0,
        #     1.0
        # )
        # self.e_buffer.append(e)
        # return self.last_control

    
#old compute control idea
# errore_vel = velocita_target - (velocita_corrente * 3.6)
# errore_vel_perc = abs(errore_vel) * 100 / velocita_target

# if errore_vel > 0:
#             throttle = 1.0 * errore_vel_perc
#         else:
#             brake = 1.0 * errore_vel_perc    
