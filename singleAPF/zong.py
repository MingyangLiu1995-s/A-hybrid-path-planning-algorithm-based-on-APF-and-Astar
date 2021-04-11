import math
import matplotlib.pyplot as plt
import time
from assistant import *
from car import *
from apf import *


# main
if __name__ == "__main__":
    print("Creating a map:\n----------")
    length = 80
    width = 80
    del_block = 2

    map = MAP(length, width, del_block)
    fig = map.create_map()

    print("\nCreating lanes:\n----------")
    solid_lines, dotted_lines, transition_lines = map.create_lanes(fig)
    print("solid lines:", solid_lines)
    print("dotted lines:", dotted_lines)
    print("transition lines:", transition_lines)

    print("\nCreating obstacles:\n----------")
    obstacles = map.get_obstacles(fig)
    print("obstacles:", obstacles)

    print("\nCreating vehicles:\n----------")
    L = 4.8
    B = 1.8
    vehicles, init_movingvehis, moving_vehicles = map.get_vehicles(fig, L, B)
    print("vehicles:", vehicles)
    print("moving vehicles:", moving_vehicles)

    print("\nSetting start and target:\n----------")
    start, target = map.get_startandtarget()
    print("start:", start, ", target:", target)

    # vehicle model
    print("\nEstablishing the vehicle model:\n----------")
    l = 3
    # b = 1.6
    velocity = 20                             # changeable
    init_force_direction = 0
    init_fi = 0                               # changeable
    init_si = init_force_direction - init_fi  # -40°~40°
    init_position = [start[0], start[1], init_fi]

    car = CAR(L, B, l, velocity)
    rear_v, rear_p, front_v, front_p, fi, vf = car.model(init_position, init_fi, init_si)
    print("initial steering angle:", int(init_si / math.pi * 180))
    print("initial rear velocity:", rear_v, ", initial rear position:", rear_p)
    print("initial front velocity:", front_v, ", initial front position:", front_p)
    print("initial vehicle direction:", int(fi / math.pi * 180))

    dif_distance, dif_distance2 = car.difference(rear_p, front_p, init_si)
    print("difference between formula and subtract:", dif_distance - dif_distance2)
    print("distance difference:", dif_distance)

    car.draw_car(rear_p, front_p)
    moving_vehicles, p1, p2, p3, = car.draw_movingcar(moving_vehicles)
    # plt.pause(0.1)
    plt.plot(start[0], start[1], '*', color='purple', markersize=10)  # drawing again
    p1.remove()
    p2.remove()
    p3.remove()

    # APF path planning algorithm
    t1 = time.time()

    print("\nAPF algorithm path planning:\n----------")
    k_att = 40
    k_rep = 10
    r_apf = 5
    k_replane = 5
    k_lane = 10
    k_car = 500
    lane_width = 3.5
    target_area = 3.5
    F_e = 50
    del_t = 0.05  # changeable
    max_iters = 105

    rear_velocities = [car.v]
    path = [[rear_p[0], rear_p[1]]]
    direction = [fi]
    front_velocities = [vf]
    front_points = [[front_p[0], front_p[1]]]
    steering = [init_si]

    apf = APF(k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, target_area, F_e, del_t, max_iters,
              solid_lines, transition_lines, obstacles, vehicles, L, B, car, rear_velocities, path, direction,
              front_velocities, front_points, steering)

    iters = 0
    rear_p, front_p, fi, iters, moving_vehicles = apf.pathplanning(rear_p, front_p, fi, target, iters, moving_vehicles)

    t2 = time.time()
    apf_time = t2 - t1

    print("total rear velocity:", apf.rear_velocities)
    print("total path:", apf.path)
    print("total direction:", apf.direction)
    print("total front velocity:", apf.front_velocities)
    print("total front position:", apf.front_points)
    print("total steering:", apf.steering)
    print("total iters:", iters)
    print("apf time:", apf_time)
    print("total distance difference:", car.dif_distances)

    print("attractive forces:", apf.F_atts)
    print("lane forces:", apf.F_lanes)
    print("obstacle forces:", apf.F_obss)
    print("vehicles forces:", apf.F_vehs)
    print("total forces:", apf.F_totals)

    map.final_draw(fig, apf.front_points, apf.path, init_movingvehis, moving_vehicles, L, B)
    plt.pause(0.1)

    plt.show()
