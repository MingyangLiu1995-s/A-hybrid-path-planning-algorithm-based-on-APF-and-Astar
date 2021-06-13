import math
import matplotlib.pyplot as plt
import time
from assistant import *
from astar import *
from car import *


# main
if __name__ == "__main__":
    print("Creating a map:\n----------")
    length = 80
    width = 80
    lane_width = 3.5
    L = 4.8
    B = 1.8

    map = MAP(length, width, lane_width, L, B)
    fig = map.create_map()

    print("\nCreating lanes:\n----------")
    solid_lines, dotted_lines, transition_lines = map.create_lanes(fig)
    print("solid lines:", solid_lines)
    print("dotted lines:", dotted_lines)
    print("transition lines:", transition_lines)
    line_areas = map.create_lineareas(solid_lines)
    print("infeasible line areas:", line_areas)

    print("\nCreating obstacles:\n----------")
    obstacles = map.get_obstacles(fig)
    print("obstacles:", obstacles)
    obstacle_areas = map.get_obsareas(obstacles)
    print("infeasible obstacle areas:", obstacle_areas)

    print("\nCreating vehicles:\n----------")
    vehicles, init_movingvehis, moving_vehicles = map.get_vehicles(fig)
    print("vehicles:", vehicles)
    print("moving vehicles:", moving_vehicles)
    vehicle_areas, moving_areas = map.get_vehareas(vehicles, moving_vehicles)
    print("infeasible vehicle areas:", vehicle_areas)
    print("infeasible moving vehicle areas:", moving_areas)

    print("\nCreating infeasible areas:\n----------")
    infeasible_areas = map.create_areas()
    print("infeasible areas:", infeasible_areas)

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
    init_si = init_force_direction - init_fi  # random
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
    moving_vehicles, moving_areas, p1, p2, p3, = car.draw_movingcar(moving_vehicles)
    # plt.pause(0.1)
    plt.plot(start[0], start[1], '*', color='purple', markersize=10)  # drawing again
    p1.remove()
    p2.remove()
    p3.remove()

    # A* path planning algorithm
    t1 = time.time()

    print("\nA* algorithm path planning:\n----------")
    target_area = 3.5
    del_t = 0.05  # changeable
    del_block = velocity * del_t
    max_iters = 1000

    rear_velocities = [car.v]
    path = [[rear_p[0], rear_p[1]]]
    direction = [fi]
    front_velocities = [vf]
    front_points = [[front_p[0], front_p[1]]]
    steering = [init_si]

    astar = ASTAR(del_block, target, target_area, del_t, max_iters, line_areas, obstacle_areas, vehicle_areas,
                  infeasible_areas, car, rear_velocities, path, direction, front_velocities, front_points, steering)

    iters = 0
    rear_p, front_p, fi, iters, moving_vehicles = astar.pathplanning(rear_p, front_p, fi, iters, moving_vehicles,
                                                                     moving_areas)

    t2 = time.time()
    astar_time = t2 - t1

    print("total rear velocity:", astar.rear_velocities)
    print("total path:", astar.path)
    print("total direction:", astar.direction)
    print("total front velocity:", astar.front_velocities)
    print("total front position:", astar.front_points)
    print("total steering:", astar.steering)
    print("total iters:", iters)
    print("astar time:", astar_time)
    print("total distance difference:", car.dif_distances)

    map.final_draw(fig, astar.front_points, astar.path, init_movingvehis, moving_vehicles, L, B)
    plt.pause(0.1)

    plt.show()
