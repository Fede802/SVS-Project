"""
Welcome to CARLA manual control with steering wheel Logitech G29.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))

import carla, carla_utility, math
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

class ControlInfo:
    def __init__(self, cc=False, ego_control=carla.VehicleControl(), other_vehicle_control=carla.VehicleControl(), min_permitted_offset=7, target_velocity=90, obstacle_relative_velocity=0):
        self.__cc = cc
        self.ego_control = ego_control
        self.other_vehicle_control = other_vehicle_control
        self.running = True
        self.min_permitted_offset = min_permitted_offset
        self.target_velocity = target_velocity
        self.reset = False
        self.obstacle_relative_velocity = obstacle_relative_velocity
        
    def __str__(self):
        return f"ego_control: {self.ego_control}, other_vehicle_control: {self.other_vehicle_control}, cc: {self.__cc}, running: {self.running}"

    def reset_ego_control(self):
        self.ego_control = carla.VehicleControl()

    def reset_other_vehicle_control(self):
        self.other_vehicle_control = carla.VehicleControl()

    def change_distance_offset(self):
        self.min_permitted_offset = (self.min_permitted_offset) % 21 + 7
        
    def increase_target_velocity(self):
        self.target_velocity += 1 if self.target_velocity < 130 else 0
    
    def decrease_target_velocity(self):
        self.target_velocity -= 1 if self.target_velocity > 30 else 0

    def cc(self):
        return self.__cc

    def set_cc(self, cc):
        self.__cc = cc
        self.reset = True

    def toggle_pid(self):
        self.set_cc(not self.__cc)

class DualControl(object):
    def __init__(self, restart_callback):
        self._restart_callback = restart_callback
        self._steer_cache = 0.0
       
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

    def parse_events(self, camera_manager, control_info, clock):
        control = control_info.ego_control
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    self._restart_callback()
                # elif event.button == 1:
                #     # world.hud.toggle_info()
                elif event.button == 2:
                    camera_manager.toggle_camera()
                elif event.button == 3:
                    carla_utility.next_weather()
                elif event.button == self._reverse_idx:
                    control.gear = 1 if control.reverse else -1
                # elif event.button == 23:
                #     world.camera_manager.next_sensor()
                elif event.button == 10:
                    # TODO: mettere toggle distanza
                    control_info.change_distance_offset()
                elif event.button == 9: 
                    control_info.toggle_pid()
                    if not control_info.cc():
                        control_info.ego_control.throttle = 0.0
                        control_info.ego_control.brake = 0.0
                    
                elif event.button == 22:
                    control_info.increase_target_velocity()
                elif event.button == 21:
                    control_info.decrease_target_velocity()

            elif event.type == pygame.KEYDOWN:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    self._restart_callback()
                elif event.key == K_TAB:
                    camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    carla_utility.next_weather(reverse=True)
                elif event.key == K_c:
                    carla_utility.next_weather()
                elif event.key == K_h:
                    control_info.change_distance_offset()
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
                elif event.key == K_p:
                    control_info.toggle_pid()
                    if not control_info.cc():
                        control_info.ego_control.throttle = 0.0
                        control_info.ego_control.brake = 0.0
                elif event.key == pygame.K_PLUS:
                    control_info.increase_target_velocity()
                elif event.key == pygame.K_MINUS:
                    control_info.decrease_target_velocity()            

        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time(), control_info)
        #self._parse_vehicle_wheel(control) #TODO "To drive start by preshing the brake pedal :')"
        control.reverse = control.gear < 0
        # world.player.apply_control(self._control)  TODO: Lasciamo commentato???????
        # control_info.ego_control = control # TODO: Dovrebbe bastare questo 
        return False

    def _parse_vehicle_keys(self, keys, milliseconds, control_info):
        control = control_info.ego_control
        control2 = control_info.other_vehicle_control
        if keys[K_w]:
            control.throttle = 1.0
            control.brake = 0.0
        if keys[K_s]:
            control.throttle = 0.0
            control.brake = 1.0
            control_info.set_cc(False)
        if keys[K_UP]:
            control2.throttle = 1.0
            control2.brake = 0.0
        if keys[K_DOWN]:
            control2.throttle = 0.0
            control2.brake = 1.0    
        control.hand_brake = keys[K_SPACE]
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        control.steer = round(self._steer_cache, 1)
        

    def _parse_vehicle_wheel(self, control):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        print (jsInputs)
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
            
        if brakeCmd != 0:
            control.toggle_pid()
        control.steer = steerCmd
        control.brake = brakeCmd
        control.throttle = throttleCmd
       
        #toggle = jsButtons[self._reverse_idx]

        control.hand_brake = bool(jsButtons[self._handbrake_idx])

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)