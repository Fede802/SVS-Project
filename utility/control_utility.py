"""
Welcome to CARLA manual control with steering wheel Logitech G29.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla, carla_utility, math
import server
from configparser import ConfigParser

import pygame
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_BACKQUOTE
from pygame.locals import K_BACKSPACE
from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_TAB
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_c
from pygame.locals import K_d
from pygame.locals import K_h
from pygame.locals import K_m
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_w

class ACCInfo:
    def __init__(self, active = False, target_velocity = None, min_permitted_offset = None, reset = False):
        self.__active = active
        self.target_velocity = target_velocity
        self.min_permitted_offset = min_permitted_offset
        self.reset = reset

    def change_distance_offset(self):
        self.min_permitted_offset = (self.min_permitted_offset) % 21 + 7
        
    def increase_target_velocity(self):
        self.target_velocity += 1 if self.target_velocity < 130 else 0
    
    def decrease_target_velocity(self):
        self.target_velocity -= 1 if self.target_velocity > 30 else 0

    def is_active(self):
        return self.__active

    def set_active(self, active):
        self.__active = active
        self.reset = True

    def toggle_acc(self): 
        self.set_active(not self.__active)
 
class ProgramInfo:
    def __init__(self, acc_info, send_info = False):
        self.acc_info = acc_info
        self.ego_control = carla.VehicleControl()
        self.other_vehicle_control = carla.VehicleControl()
        self.running = True
        self.ego_velocity = 0.0
        self.obstacle_relative_velocity = 0.0
        self.send_info = send_info
        
    def __str__(self):
        return f"ego_control: {self.ego_control}, other_vehicle_control: {self.other_vehicle_control}, cc: {self.accInfo.is_active()}, running: {self.running}"

class DualControl(object):
    def __init__(self, restart_callback):
        self._restart_callback = restart_callback
        self._ego_steer_cache = 0.0
        self._other_steer_cache = 0.0
        self._reverse_cache = False
       
        # initialize steering wheel
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")
        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('.\wheel_config.ini')
        self._steer_idx = int(self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(self._parser.get('G29 Racing Wheel', 'handbrake'))

    def parse_events(self, camera_manager, program_info, clock):
        control = program_info.ego_control
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 1: #X
                    self._restart_callback(1)
                elif event.button == 3: #Y
                    self._restart_callback(2)
                elif event.button == 0: #A
                    self._restart_callback(3) 
                elif event.button == 2: #B
                    self._restart_callback(4)             
                # elif event.button == 1:
                #     # world.hud.toggle_info()
                elif event.button == 8:
                    camera_manager.toggle_camera()
                elif event.button == 11:
                    carla_utility.next_weather()
                    program_info.send_info and server.send_data("Weather Changed")
                elif event.button == self._reverse_idx:
                    self._reverse_cache = not self._reverse_cache
                    #control.gear = 1 if control.reverse else -1
                
               
                # elif event.button == 23:
                #     world.camera_manager.next_sensor()
                elif event.button == 9:
                    program_info.acc_info.toggle_acc()
                    program_info.acc_info.target_velocity = program_info.ego_velocity
                    if not program_info.acc_info.is_active():
                        control.throttle = 0.0
                        control.brake = 0.0
                    program_info.send_info and server.send_data(f"ACC Toggled to {program_info.acc_info.is_active()}")
                elif event.button == 10:
                    # TODO: mettere toggle distanza
                    program_info.acc_info.change_distance_offset()
                    program_info.send_info and server.send_data(f"Distance Changed to {program_info.acc_info.min_permitted_offset}")
                elif event.button == 27: 
                    program_info.acc_info.toggle_acc()
                    program_info.acc_info.target_velocity = 90
                    if not program_info.acc_info.is_active():
                        control.throttle = 0.0
                        control.brake = 0.0
                    program_info.send_info and server.send_data(f"ACC Toggled to {program_info.acc_info.is_active()}")
                elif event.button == 21:
                    program_info.acc_info.increase_target_velocity()
                    program_info.send_info and server.send_data(f"Target Velocity Changed to {program_info.acc_info.target_velocity}")
                elif event.button == 22:
                    program_info.acc_info.decrease_target_velocity()
                    program_info.send_info and server.send_data(f"Target Velocity Changed to {program_info.acc_info.target_velocity}")

            elif event.type == pygame.KEYDOWN:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == pygame.K_1:
                    self._restart_callback(1)
                elif event.key == pygame.K_2:
                    self._restart_callback(2)
                elif event.key == pygame.K_3:
                    self._restart_callback(3)
                elif event.key == pygame.K_4:
                    self._restart_callback(4)    
                elif event.key == K_TAB:
                    camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    carla_utility.next_weather(reverse=True)
                    program_info.send_info and server.send_data("Weather Changed")
                elif event.key == K_c:
                    carla_utility.next_weather()
                    program_info.send_info and server.send_data("Weather Changed")
                elif event.key == K_h:
                    program_info.acc_info.change_distance_offset()
                    program_info.send_info and server.send_data(f"Distance Changed to {program_info.acc_info.min_permitted_offset}")
                if event.key == K_q:
                    control.gear = 1 if control.reverse else -1
                elif event.key == K_m:
                    control.manual_gear_shift = not control.manual_gear_shift
                    control.gear = control.gear
                    # world.hud.notification('%s Transmission' %('Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif control.manual_gear_shift and event.key == K_COMMA:
                    control.gear = max(-1, control.gear - 1)
                elif control.manual_gear_shift and event.key == K_PERIOD:
                    control.gear = control.gear + 1
                elif event.key == K_p or event.key == pygame.K_o:
                    program_info.acc_info.toggle_acc()
                    if event.key == pygame.K_p:
                        program_info.acc_info.target_velocity = 90
                    else:
                        program_info.acc_info.target_velocity = program_info.ego_velocity
                    if not program_info.acc_info.is_active():
                        control.throttle = 0.0
                        control.brake = 0.0
                    program_info.send_info and server.send_data(f"ACC Toggled to {program_info.acc_info.is_active()}")
                elif event.key == pygame.K_PLUS:
                    program_info.acc_info.increase_target_velocity()
                    program_info.send_info and server.send_data(f"Target Velocity Changed to {program_info.acc_info.target_velocity}")
                elif event.key == pygame.K_MINUS:
                    program_info.acc_info.decrease_target_velocity()  
                    program_info.send_info and server.send_data(f"Target Velocity Changed to {program_info.acc_info.target_velocity}")          
        
        
        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time(), program_info)
        # self._parse_vehicle_wheel(program_info) #TODO "To drive start by preshing the brake pedal :')"
        control.reverse = control.gear < 0
        # world.player.apply_control(self._control)  TODO: Lasciamo commentato???????
        # control_info.ego_control = control # TODO: Dovrebbe bastare questo 
        program_info.ego_control.reverse = self._reverse_cache
        return False

    def _parse_vehicle_keys(self, keys, milliseconds, program_info):
        control = program_info.ego_control
        control2 = program_info.other_vehicle_control
        if keys[K_w]:
            control.throttle = 1.0
            control.brake = 0.0
        if keys[K_s]:
            control.throttle = 0.0
            control.brake = 1.0
            program_info.acc_info.set_active(False)
        control.hand_brake = keys[K_SPACE]
        steer_increment = 5e-4 * milliseconds
        if keys[K_a]:
            self._ego_steer_cache -= steer_increment
        elif keys[K_d]:
            self._ego_steer_cache += steer_increment
        else:
            self._ego_steer_cache = 0.0
        self._ego_steer_cache = min(0.7, max(-0.7, self._ego_steer_cache))
        control.steer = round(self._ego_steer_cache, 1)    

        if keys[K_UP]:
            control2.throttle = 1.0
            control2.brake = 0.0
        if keys[K_DOWN]:
            control2.throttle = 0.0
            control2.brake = 1.0    
        
        if keys[K_LEFT]:
            self._other_steer_cache -= steer_increment
        elif keys[K_RIGHT]:
            self._other_steer_cache += steer_increment
        else:
            self._other_steer_cache = 0.0    
        self._other_steer_cache = min(0.7, max(-0.7, self._other_steer_cache))
        control2.steer = round(self._other_steer_cache, 1)
        

    def _parse_vehicle_wheel(self, program_info):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        #throttleCmd = K2 + (2.05 * math.log10(
        #    -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        K2 = 1.6  # 1.6
        #throttleCmd = K2 + (2.05 * math.log10(
        #    -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if jsInputs[self._throttle_idx] == 0.0:
                throttleCmd = 0
        else:
            throttleCmd = K2 + (2.05 * math.log10(
                0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
            if throttleCmd <= 0:
                throttleCmd = 0
            elif throttleCmd > 1:
                throttleCmd = 1

           

        #brakeCmd = 1.6 + (2.05 * math.log10(
        #    -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        brakeCmd = 1.6 + (2.05 * math.log10(
            0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1
            
            
        program_info.ego_control.steer = steerCmd
        if jsInputs[self._brake_idx] != -1:
            program_info.acc_info.set_active(False)
            program_info.ego_control.brake = brakeCmd
        if jsInputs[self._throttle_idx] != -1:
            program_info.ego_control.throttle = throttleCmd
       
        #toggle = jsButtons[self._reverse_idx]

        program_info.ego_control.hand_brake = bool(jsButtons[self._handbrake_idx])

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)