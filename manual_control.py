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
    def __init__(self, carla_world, actor_filter):
        self.world = carla_world
        self.player = None
        self.collision_sensor = None
        # self.lane_invasion_sensor = None
        # self.gnss_sensor = None
        self.camera_manager = None 
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        # self.world.on_tick(hud.on_world_tick)

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get the cybertruck blueprint.
        blueprint = self.world.get_blueprint_library().find('vehicle.tesla.cybertruck')
        # blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_point = carla.Transform(carla.Location(x=2388, y=6164, z=187), carla.Rotation(yaw = -88.2))
            self.player = carla_utility.spawn_vehicle_bp_at(world=self.world, vehicle='vehicle.tesla.cybertruck', spawn_point=spawn_point)

        # Set up the sensors.
        # self.collision_sensor = CollisionSensor(self.player, self.hud)
        # self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        # self.gnss_sensor = GnssSensor(self.player)
        # self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, None)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        # self.hud.notification(actor_type)

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

    def render(self, display):
        self.camera_manager.render(display)
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
        self._autopilot_enabled = start_in_autopilot
        self._control_info = control_info
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
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
        self._steer_idx = int(
            self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))

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

            elif event.type == pygame.KEYUP:
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
                if isinstance(self._control, carla.VehicleControl):
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
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            # world.player.apply_control(self._control)  TODO: Lasciamo commentato???????
            self._control_info.ego_control = self._control # TODO: Dovrebbe bastare questo 

    def _parse_vehicle_keys(self, keys, milliseconds):
        if not self._control_info.pid_cc:
            print("PID CC is not enabled")
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
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        #throttleCmd = K2 + (2.05 * math.log10(
        #    -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if not self._control_info.pid_cc:
            throttleCmd = K2 + (2.05 * math.log10(0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
            brakeCmd = 1.6 + (2.05 * math.log10(
            0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
            if brakeCmd <= 0:
                brakeCmd = 0
            elif brakeCmd > 1:
                brakeCmd = 1
            self._control.steer = steerCmd
            self._control.brake = brakeCmd
            self._control.throttle = throttleCmd
       
        # if throttleCmd <= 0:
        #     throttleCmd = 0
        # elif throttleCmd > 1:
        #     throttleCmd = 1

        #brakeCmd = 1.6 + (2.05 * math.log10(
        #    -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92


        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

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
        self.recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            # if item[0].startswith('sensor.camera'):
            #     bp.set_attribute('image_size_x', str(hud.dim[0]))
            #     bp.set_attribute('image_size_y', str(hud.dim[1]))
            # elif item[0].startswith('sensor.lidar'):
            #     bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


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


# # ==============================================================================
# # -- main() --------------------------------------------------------------------
# # ==============================================================================


# def main():
#     argparser = argparse.ArgumentParser(
#         description='CARLA Manual Control Client')
#     argparser.add_argument(
#         '-v', '--verbose',
#         action='store_true',
#         dest='debug',
#         help='print debug information')
#     argparser.add_argument(
#         '--host',
#         metavar='H',
#         default='127.0.0.1',
#         help='IP of the host server (default: 127.0.0.1)')
#     argparser.add_argument(
#         '-p', '--port',
#         metavar='P',
#         default=2000,
#         type=int,
#         help='TCP port to listen to (default: 2000)')
#     argparser.add_argument(
#         '-a', '--autopilot',
#         action='store_true',
#         help='enable autopilot')
#     argparser.add_argument(
#         '--res',
#         metavar='WIDTHxHEIGHT',
#         default='1280x720',
#         help='window resolution (default: 1280x720)')
#     argparser.add_argument(
#         '--filter',
#         metavar='PATTERN',
#         default='vehicle.*',
#         help='actor filter (default: "vehicle.*")')
#     args = argparser.parse_args()

#     args.width, args.height = [int(x) for x in args.res.split('x')]

#     log_level = logging.DEBUG if args.debug else logging.INFO
#     logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

#     logging.info('listening to server %s:%s', args.host, args.port)

#     print(__doc__)

#     try:

#         game_loop(args)

#     except KeyboardInterrupt:
#         print('\nCancelled by user. Bye!')





def compute_control(control_info: ControlInfo):

    for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.QUIT or event.key == pygame.K_ESCAPE:
                    control_info.running = False
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        control_info.target_control.throttle = 1.0
        control_info.target_control.brake = 0.0
    if keys[pygame.K_s]:
        control_info.target_control.brake = 1.0
        control_info.target_control.throttle = 0.0
    if keys[pygame.K_a]:
        control_info.target_control.steer = -0.38
    if keys[pygame.K_d]:
        control_info.target_control.steer = 0.38
    if keys[pygame.K_e]:
        control_info.target_control.hand_brake = True
    if keys[pygame.K_p]:
        control_info.pid_cc = not control_info.pid_cc    
    if keys[pygame.K_UP]:
        control_info.ego_control.throttle = 1.0
        control_info.ego_control.brake = 0.0
    if keys[pygame.K_DOWN]:
        control_info.ego_control.brake = 1.0
        control_info.ego_control.throttle = 0.0
        control_info.pid_cc = False
    if keys[pygame.K_LEFT]:
        control_info.ego_control.steer = -0.38
    if keys[pygame.K_RIGHT]:
        control_info.ego_control.steer = 0.38
    if keys[pygame.K_PLUS]:
        print("Increasing min permitted distance")
        control_info.change_distance_offset()

    