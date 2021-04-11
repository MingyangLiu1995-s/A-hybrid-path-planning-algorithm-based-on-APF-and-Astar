import matplotlib.pyplot as plt
from assistant import *
from apf import *


# main
if __name__ == "__main__":
    print("Creating a map:\n----------")
    length = 50
    width = 30
    del_block = 0.25  # 0.5

    map = MAP(length, width, del_block)
    fig = map.create_map()

    print("\nCreating lanes:\n----------")
    solid_lines, dotted_lines = map.create_lanes()
    print("solid lines:", solid_lines)
    print("dotted lines:", dotted_lines)

    print("\nCreating obstacles:\n----------")
    obstacles = map.get_obstacles(fig)
    print("obstacles:", obstacles)

    print("\nCreating vehicles:\n----------")
    L = 4.8
    B = 1.8
    vehicles = map.get_vehicles(fig, L, B)
    print("vehicles:", vehicles)

    print("\nCreating effective area:\n----------")
    effective_area, infeasible_areas, height = map.create_areas(solid_lines)
    print("the effective area:", effective_area)
    print("the infeasible areas:", infeasible_areas)

    print("\nCreating nodes:\n----------")
    nodes, useless_nodes = map.get_nodes(effective_area, infeasible_areas, height)
    print("nodes:", nodes)
    print("useless nodes:", useless_nodes)

    print("\nSetting start and target:\n----------")
    start, target = map.get_startandtarget()
    print("start:", start, ", target:", target)

    print("\n----------")

    k_att = 40
    k_rep = 10
    r_apf = 5
    k_replane = 5
    k_lane = 10
    k_car = 500
    lane_width = 3.5
    L = 4.8
    B = 1.8

    apf = APF(k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, L, B)

    U_att = map.appear_attraction(nodes, useless_nodes, target, apf)
    U_lane = map.appear_lanerepulsion(nodes, useless_nodes, solid_lines, apf)
    U_obs = map.appear_obsrepulsion(nodes, useless_nodes, obstacles, apf)
    U_veh = map.appear_vehirepulsion(nodes, useless_nodes, vehicles, apf)
    U_total = map.total_potential(nodes, useless_nodes, U_att, U_lane, U_obs, U_veh)

    plt.show()
