import pygame
import carla

class ControlInfo:
    def __init__(self):
        self.running = True
        self.ego_control = carla.VehicleControl()
        self.target_control = carla.VehicleControl()

def compute_control():
    control_info = ControlInfo()
    for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.QUIT or event.key == pygame.K_ESCAPE:
                    control_info.running = False
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        control_info.target_control.throttle = 1.0
    if keys[pygame.K_s]:
        control_info.target_control.brake = 1.0
    if keys[pygame.K_a]:
        control_info.target_control.steer = -0.38
    if keys[pygame.K_d]:
        control_info.target_control.steer = 0.38
    if keys[pygame.K_e]:
        control_info.target_control.hand_brake = True
    if keys[pygame.K_UP]:
        control_info.ego_control.throttle = 1.0
    if keys[pygame.K_DOWN]:
        control_info.ego_control.brake = 1.0
    if keys[pygame.K_LEFT]:
        control_info.ego_control.steer = -0.38
    if keys[pygame.K_RIGHT]:
        control_info.ego_control.steer = 0.38
    
    return control_info