import math
import matplotlib.pyplot as plt
from car import *


class ASTAR:

    class NODE:

        def __init__(self, current_node, father_node, target, cost=0):
            self.current_node = current_node
            self.father_node = father_node
            self.target = target
            self.direction = math.atan2(self.current_node[1] - self.father_node[1],
                                        self.current_node[0] - self.father_node[0])
            self.g = cost
            self.h = abs(self.target[0] - self.current_node[0]) + abs(self.target[1] - self.current_node[1])
            self.f = self.g + self.h

    def __init__(self, del_block, target, target_area, del_t, max_iters, line_areas, obstacle_areas, vehicle_areas,
                 infeasible_areas, car, rear_velocities, path, direction, front_velocities, front_points, steering):
        self.del_block = del_block
        self.target = target
        self.target_area = target_area
        self.del_t = del_t
        self.max_iters = max_iters
        self.line_areas = line_areas
        self.obstacle_areas = obstacle_areas
        self.vehicle_areas = vehicle_areas
        self.infeasible_areas = infeasible_areas
        self.car = car
        self.rear_velocities = rear_velocities
        self.path = path
        self.direction = direction
        self.front_velocities = front_velocities
        self.front_points = front_points
        self.steering = steering

    def possible_moves(self, current_node, moving_areas):
        next_nodes, next_costs = [], []

        possible_nodes = [[current_node[0] + self.del_block, current_node[1]],
                          [current_node[0] + self.del_block, current_node[1] + self.del_block],
                          [current_node[0], current_node[1] + self.del_block],
                          [current_node[0] - self.del_block, current_node[1] + self.del_block],
                          [current_node[0] - self.del_block, current_node[1]],
                          [current_node[0] - self.del_block, current_node[1] - self.del_block],
                          [current_node[0], current_node[1] - self.del_block],
                          [current_node[0] + self.del_block, current_node[1] - self.del_block]]
        possiblenodes2 = [[current_node[0] + self.del_block, current_node[1]],
                          [current_node[0] + self.del_block, current_node[1] + self.del_block],
                          [current_node[0], current_node[1] + self.del_block],
                          [current_node[0] - self.del_block, current_node[1] + self.del_block],
                          [current_node[0] - self.del_block, current_node[1]],
                          [current_node[0] - self.del_block, current_node[1] - self.del_block],
                          [current_node[0], current_node[1] - self.del_block],
                          [current_node[0] + self.del_block, current_node[1] - self.del_block]]

        possible_costs = [self.del_block, math.hypot(self.del_block, self.del_block),
                          self.del_block, math.hypot(self.del_block, self.del_block),
                          self.del_block, math.hypot(self.del_block, self.del_block),
                          self.del_block, math.hypot(self.del_block, self.del_block)]
        possiblecosts2 = [self.del_block, math.hypot(self.del_block, self.del_block),
                          self.del_block, math.hypot(self.del_block, self.del_block),
                          self.del_block, math.hypot(self.del_block, self.del_block),
                          self.del_block, math.hypot(self.del_block, self.del_block)]

        areas = self.line_areas + self.obstacle_areas + self.infeasible_areas + self.vehicle_areas + moving_areas

        for area in areas:
            for i in range(len(possiblenodes2)):
                if area[0] <= possiblenodes2[i][0] <= area[2] and area[1] <= possiblenodes2[i][1] <= area[3]:
                    if possiblenodes2[i] in possible_nodes:
                        possible_nodes.remove(possiblenodes2[i])
                        possible_costs.remove(possiblecosts2[i])

        for i in range(len(possible_nodes)):
            next_nodes.append(possible_nodes[i])
            next_costs.append(possible_costs[i])

        return next_nodes, next_costs

    def get_minf(self, best, next):
        if next.f < best.f:
            best = next

        return best

    def pathplanning(self, rear_p, front_p, fi, iters, moving_vehicles, moving_areas):
        father_node = [front_p[0], front_p[1]]
        current_node = [front_p[0], front_p[1]]
        current = ASTAR.NODE(current_node, father_node, self.target)

        distance = math.sqrt((self.target[0] - rear_p[0]) ** 2 + (self.target[1] - rear_p[1]) ** 2)

        while distance > self.target_area and iters <= self.max_iters:
            iters += 1
            print("\n----------\nNo.", iters)
            print("current node:", current.current_node, ", previous angle:", current.direction, ", current g:",
                  current.g, ", h:", current.h, ", f:", current.f)

            next_nodes, next_costs = self.possible_moves(current.current_node, moving_areas)

            next_set = []
            for next_node, next_cost in zip(next_nodes, next_costs):
                next = ASTAR.NODE(next_node, current.current_node, self.target, next_cost + current.g)
                next_set.append(next)

            current.f = float("inf")
            best = current
            for next in next_set:
                best = self.get_minf(best, next)

            if best == current:
                print("\n----------\nA* is failed!\n----------")
                exit()

            current_node = best.current_node
            father_node = best.father_node
            current_cost = best.g
            current = ASTAR.NODE(current_node, father_node, self.target, current_cost)
            si = current.direction - fi
            print("steering angle:", int(si / math.pi * 180))

            rear_v, rear_p, front_v, front_p, fi, vf = self.car.model(rear_p, fi, si, self.del_t)
            print("rear velocity:", rear_v, ", rear position:", rear_p)
            # print("front velocity:", front_v, ", front position:", front_p)
            # print("vehicle direction:", int(fi / math.pi * 180))

            dif_distance, dif_distance2 = self.car.difference(rear_p, front_p, si, self.del_t)
            print("difference between formula and subtract:", dif_distance - dif_distance2)
            print("distance difference:", dif_distance)

            self.car.draw_car(rear_p, front_p)
            moving_vehicles, moving_areas, p1, p2, p3, = self.car.draw_movingcar(moving_vehicles, self.del_t)
            # plt.pause(0.1)
            p1.remove()
            p2.remove()
            p3.remove()

            self.rear_velocities.append(self.car.v)
            self.path.append([rear_p[0], rear_p[1]])
            self.direction.append(fi)
            self.front_velocities.append(vf)
            self.front_points.append([front_p[0], front_p[1]])
            self.steering.append(si)

            father_node = current.father_node
            current_node = [front_p[0], front_p[1]]
            current = ASTAR.NODE(current_node, father_node, self.target, current_cost)

            distance = math.sqrt((self.target[0] - rear_p[0]) ** 2 + (self.target[1] - rear_p[1]) ** 2)

        if distance <= self.target_area and iters <= self.max_iters:
            print("\n----------\nA* is successful!\n----------")
        else:
            print("\n----------\nA* is failed!\n----------")

        return rear_p, front_p, fi, iters, moving_vehicles

