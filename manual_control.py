#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control with steering wheel Logitech G29.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'utility'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'mqtt_service'))

import carla_utility # type: ignore

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc
import cv2

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import hud

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

try:
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
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


class ControlInfo:
    def __init__(self, pid_cc=False, ego_control=carla.VehicleControl(), target_control=carla.VehicleControl(), min_permitted_offset=7, target_velocity=90):
        self.pid_cc = pid_cc
        self.ego_control = ego_control
        self.target_control = target_control
        self.running = True
        self.min_permitted_offset = min_permitted_offset
        self.target_velocity = target_velocity
        

    def __str__(self):
        return f"ego_control: {self.ego_control}, target_control: {self.target_control}, pid_cc: {self.pid_cc}, running: {self.running}"

    def reset_ego_control(self):
        self.ego_control = carla.VehicleControl()

    def change_distance_offset(self):
        if self.min_permitted_offset == 7 or self.min_permitted_offset == 14:
            self.min_permitted_offset += 7
        else:
            self.min_permitted_offset = 7

    def increase_target_velocity(self):
        self.target_velocity += 10 if self.target_velocity < 130 else 0
    
    def decrease_target_velocity(self):
        self.target_velocity -= 10 if self.target_velocity > 50 else 0

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name




# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world):
        self.world = carla_world
        self.spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=187), carla.Rotation(yaw = -88.2))
        self.player = None
        self.collision_sensor = None
        self.camera_manager = None 
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.restart()
        # self.world.on_tick(hud.on_world_tick)

    def restart(self):
        carla_utility.destroy_all_vehicle_and_sensors(self.world)
        print("Restarting world")
        # Keep same camera config if the camera manager exists.
        # Set up the sensors.
        # self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.player = carla_utility.spawn_vehicle_bp_at(world=self.world, vehicle='vehicle.tesla.cybertruck', spawn_point=self.spawn_point)
        # self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, None)
        print("Restarting world")
       
        

    def apply_control(self, control):
        self.player.apply_control(control)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        # self.hud.tick(self, clock)
        # TODO: mettere qua l'aggiornamento delle scritte
        return

    def render(self, display, control_info):
        self.camera_manager.render(display, control_info)
        # self.hud.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            # self.collision_sensor.sensor,
            # self.lane_invasion_sensor.sensor,
            # self.gnss_sensor.sensor
            ]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot, control_info):
        print("DualControl init")
        self._autopilot_enabled = start_in_autopilot
        self._control_info = control_info
        self._control = carla.VehicleControl()
        # world.player.set_autopilot(self._autopilot_enabled)
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

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                elif event.button == 1:
                    world.hud.toggle_info()
                # elif event.button == 2:
                #     world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                # elif event.button == 23:
                #     world.camera_manager.next_sensor()
                elif event.button == 10:
                    # TODO: mettere toggle distanza
                    self._control_info.change_distance_offset()
                elif event.button == 9: 
                    self._control_info.pid_cc = not self._control_info.pid_cc
                elif event.button == 22:
                    self._control_info.increase_target_velocity()
                elif event.button == 21:
                    self._control_info.decrease_target_velocity()

            elif event.type == pygame.KEYDOWN:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self._control.gear = world.player.get_control().gear
                    world.hud.notification('%s Transmission' %
                                            ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        if not self._autopilot_enabled:
            
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            #self._parse_vehicle_wheel() #TODO "To drive start by preshing the brake pedal :')"
            self._control.reverse = self._control.gear < 0
            # world.player.apply_control(self._control)  TODO: Lasciamo commentato???????
            self._control_info.ego_control = self._control # TODO: Dovrebbe bastare questo 

    def _parse_vehicle_keys(self, keys, milliseconds):
        # if not self._control_info.pid_cc:
        #     # print("PID CC is not enabled")
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = 1.0
            self._control_info.pid_cc = False
        # self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
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

    
            
        print(f"Steer: {steerCmd}, Throttle: {throttleCmd}, Brake: {brakeCmd}")
        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd
       
        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# # ==============================================================================
# # -- CameraManager -------------------------------------------------------------
# # ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud=None):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        # self.hud = hud
        self.sensors = ['sensor.camera.rgb', cc.Raw, 'Camera RGB']
        self.sensors.append(self._parent.get_world().get_blueprint_library().find(self.sensors[0]))
        self.sensors[-1].set_attribute('image_size_x', "1280")
        self.sensors[-1].set_attribute('image_size_y', "720")
            # if item[0].startswith('sensor.camera'):
            #     bp.set_attribute('image_size_x', str(hud.dim[0]))
            #     bp.set_attribute('image_size_y', str(hud.dim[1]))

        self.sensor = self._parent.get_world().spawn_actor(
            self.sensors[-1],
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))    

    #change camera view
    
    def print_text_to_screen(self, display, text, position, color):
        font = pygame.font.Font(None, 36)  # You can choose the font and size
        text_surface = font.render(text, True, color)  # White color
        display.blit(text_surface, position)

    def toggle_recording(self):
        self.recording = not self.recording

    def render(self, display, control_info):
        if self.surface is not None:
            scaled_surface = pygame.transform.scale(self.surface, display.get_size())
            display.blit(scaled_surface, (0, 0))
        self.print_text_to_screen(display, f"Throttle: {control_info.ego_control.throttle}", (10, 10), (255, 255, 255))
        self.print_text_to_screen(display, f"Brake: {control_info.ego_control.brake}", (10, 50), (255, 255, 255))
        self.print_text_to_screen(display, f"Steer: {control_info.ego_control.steer}", (10, 90), (255, 255, 255))
        self.print_text_to_screen(display, f"Hand Brake: {control_info.ego_control.hand_brake}", (10, 130), (255, 255, 255))
        self.print_text_to_screen(display, f"PID CC: {control_info.pid_cc}", (10, 170), (255, 255, 255))
        self.print_text_to_screen(display, f"Min Permitted Distance: {control_info.min_permitted_offset}", (10, 210), (255, 255, 255))
        self.print_text_to_screen(display, f"Target Velocity: {control_info.target_velocity}", (10, 250), (255, 255, 255))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


# def game_loop(args):
#     pygame.init()
#     pygame.font.init()
#     world = None

#     try:
#         client = carla.Client(args.host, args.port)
#         client.set_timeout(2.0)

#         #display = pygame.display.set_mode(
#         #    (args.width, args.height),
#         #    pygame.HWSURFACE | pygame.DOUBLEBUF)
#         display = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)

#         hud = HUD(args.width, args.height)
#         world = World(client.get_world(), hud, args.filter)
#         controller = DualControl(world, args.autopilot)

#         clock = pygame.time.Clock()
#         while True:
#             clock.tick_busy_loop(120)
#             if controller.parse_events(world, clock):
#                 return
#             world.tick(clock)
#             world.render(display)
#             pygame.display.flip()

#     finally:

#         if world is not None:
#             world.destroy()

#         pygame.quit()

# def compute_control(control_info: ControlInfo):

#     for event in pygame.event.get():
#             if event.type == pygame.KEYDOWN:
#                 if event.key == pygame.QUIT or event.key == pygame.K_ESCAPE:
#                     control_info.running = False
#     keys = pygame.key.get_pressed()
#     if keys[pygame.K_w]:
#         control_info.target_control.throttle = 1.0
#         control_info.target_control.brake = 0.0
#     if keys[pygame.K_s]:
#         control_info.target_control.brake = 1.0
#         control_info.target_control.throttle = 0.0
#     if keys[pygame.K_a]:
#         control_info.target_control.steer = -0.38
#     if keys[pygame.K_d]:
#         control_info.target_control.steer = 0.38
#     if keys[pygame.K_e]:
#         control_info.target_control.hand_brake = True
#     if keys[pygame.K_p]:
#         control_info.pid_cc = not control_info.pid_cc    
#     if keys[pygame.K_UP]:
#         control_info.ego_control.throttle = 1.0
#         control_info.ego_control.brake = 0.0
#     if keys[pygame.K_DOWN]:
#         control_info.ego_control.brake = 1.0
#         control_info.ego_control.throttle = 0.0
#         control_info.pid_cc = False
#     if keys[pygame.K_LEFT]:
#         control_info.ego_control.steer = -0.38
#     if keys[pygame.K_RIGHT]:
#         control_info.ego_control.steer = 0.38
#     if keys[pygame.K_PLUS]:
#         print("Increasing min permitted distance")
#         control_info.change_distance_offset()

    