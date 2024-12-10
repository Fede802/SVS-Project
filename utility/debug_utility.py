import math, carla, math_utility

def draw_radar_bounding_range(radar: carla.Actor, life_time = 0.1, color = carla.Color(255, 0, 0)):
    debug = radar.get_world().debug
    radar_h_fov = math.radians(int(radar.attributes.get('horizontal_fov')))
    radar_v_fov = math.radians(int(radar.attributes.get('vertical_fov')))
    radar_range = int(radar.attributes.get('range'))
    radar_location = radar.get_location()

    shifted_max_range_point = math_utility.sub(get_point_at(radar, radar_location, radar_range), radar_location)

    center = math_utility.add(shifted_max_range_point, radar_location)
    p1 = math_utility.add(math_utility.rotate(shifted_max_range_point, radar_h_fov, radar_v_fov), radar_location)
    p2 = math_utility.add(math_utility.rotate(shifted_max_range_point, radar_h_fov, -radar_v_fov), radar_location)
    p3 = math_utility.add(math_utility.rotate(shifted_max_range_point, -radar_h_fov, radar_v_fov), radar_location)
    p4 = math_utility.add(math_utility.rotate(shifted_max_range_point, -radar_h_fov, -radar_v_fov), radar_location)

    debug.draw_line(radar_location, center, life_time=life_time, color=color)
    debug.draw_line(radar_location, p1, life_time=life_time, color=color)
    debug.draw_line(radar_location, p2, life_time=life_time, color=color)
    debug.draw_line(radar_location, p3, life_time=life_time, color=color)
    debug.draw_line(radar_location, p4, life_time=life_time, color=color)

def get_point_at(radar: carla.Actor, start: carla.Vector3D, distance: int):
    radar_f_vector = radar.get_transform().get_forward_vector()
    return math_utility.vector_scalar_op(lambda a, b: a + distance * b, start, radar_f_vector)

def draw_radar_point(radar: carla.Actor, point: carla.RadarDetection, life_time = 0.1, color = carla.Color(0, 255, 0)):
    debug = radar.get_world().debug
    radar_location = radar.get_location()
    shifted_max_range_point = math_utility.sub(get_point_at(radar, radar_location, point.depth), radar_location)
    debug.draw_line(radar_location, math_utility.add(math_utility.rotate(shifted_max_range_point, point.azimuth, point.altitude),radar_location), life_time=life_time, color=color)

 

def draw_radar_point_cloud_range(radar: carla.Actor, h_radius, v_radius, life_time = 0.1, color = carla.Color(255, 0, 0)):
    debug = radar.get_world().debug
    radar_range = int(radar.attributes.get('range'))
    baseVersor1, baseVersor2 = math_utility.get_plane_base_versor_from_normal_versor(radar.get_transform().get_forward_vector())
    
    baseVersor1Shift = math_utility.vector_scalar_op(lambda a, b: h_radius * a, baseVersor1)
    baseVersor2Shift = math_utility.vector_scalar_op(lambda a, b: v_radius * a, baseVersor2)
    
    p1 = math_utility.add(math_utility.add(radar.get_location(), baseVersor1Shift),  baseVersor2Shift)
    p2 = math_utility.sub(math_utility.add(radar.get_location(), baseVersor1Shift),  baseVersor2Shift)
    p3 = math_utility.sub(math_utility.sub(radar.get_location(), baseVersor1Shift),  baseVersor2Shift)
    p4 = math_utility.add(math_utility.sub(radar.get_location(), baseVersor1Shift),  baseVersor2Shift)

    p5 = get_point_at(radar, p1, radar_range)
    p6 = get_point_at(radar, p2, radar_range)
    p7 = get_point_at(radar, p3, radar_range)
    p8 = get_point_at(radar, p4, radar_range)
    
    debug.draw_line(p1, p5, life_time=life_time, color=color)
    debug.draw_line(p2, p6, life_time=life_time, color=color)
    debug.draw_line(p3, p7, life_time=life_time, color=color)
    debug.draw_line(p4, p8, life_time=life_time, color=color)
   
def evaluate_point(radar: carla.Actor, detection: carla.RadarDetection, h_radius, v_radius):
    radar_range = int(radar.attributes.get('range'))

    x = detection.depth * math.cos(detection.azimuth) * math.cos(detection.altitude)
    y = detection.depth * math.sin(detection.azimuth) * math.cos(detection.altitude)
    z = -detection.depth * math.cos(detection.azimuth) * math.sin(detection.altitude) 

    return 0 < x < radar_range and -h_radius < y < h_radius and -v_radius < z < v_radius

def get_point_from_trasform(transform: carla.Transform, distance: int):
    radar_f_vector = transform.get_forward_vector()
    return math_utility.vector_scalar_op(lambda a, b: a + distance * b, transform.location, radar_f_vector)

def get_velocity_vector(velocity, direction: carla.Rotation):
    return carla.Vector3D(velocity * math.cos(math.radians(direction.yaw)), velocity * math.sin(math.radians(direction.yaw)), 0)