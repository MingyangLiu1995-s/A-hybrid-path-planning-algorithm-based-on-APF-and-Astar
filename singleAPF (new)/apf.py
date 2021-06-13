import math
import matplotlib.pyplot as plt
from car import *
from supplement import *


class APF:

    def __init__(self, k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, target_area, F_e, del_t, max_iters,
                 solid_lines, transition_lines, obstacles, vehicles, L, B, car, rear_velocities, path,
                 direction, front_velocities, front_points, steering):
        self.k_att = k_att
        self.k_rep = k_rep
        self.r_apf = r_apf
        self.k_replane = k_replane
        self.k_lane = k_lane
        self.k_car = k_car
        self.lw = lane_width
        self.target_area = target_area
        self.F_e = F_e
        self.del_t = del_t
        self.max_iters = max_iters
        self.h_sollines, self.v_sollines = self.solid(solid_lines)
        self.t_lines = transition_lines
        self.obstacles = obstacles
        self.vehicles = vehicles
        self.L = L
        self.B = B
        self.car_Lmin = 2 * L
        self.car_Bmin = B
        self.car = car
        self.rear_velocities = rear_velocities
        self.path = path
        self.direction = direction
        self.front_velocities = front_velocities
        self.front_points = front_points
        self.steering = steering
        self.F_atts = []
        self.F_lanes = []
        self.F_obss = []
        self.F_vehs = []
        self.F_totals = []

    def solid(self, solid_lines):
        h_sollines, v_sollines = [], []

        for solid_line in solid_lines:
            if solid_line[3] == 'h':
                h_sollines.append(solid_line)
            else:
                v_sollines.append(solid_line)

        return h_sollines, v_sollines

    def attraction(self, front_p, target):
        force_att = self.k_att
        theta_att = math.atan2(target[1] - front_p[1], target[0] - front_p[0])
        F_attx = force_att * math.cos(theta_att)
        F_atty = force_att * math.sin(theta_att)
        F_att = [F_attx, F_atty]

        return F_att

    def keeplane(self, front_p, supply):
        use_sollinesx, use_sollinesy = supply.useful_lines(front_p)

        if use_sollinesx != [] and use_sollinesy != []:
            F_lane = supply.lane_xy(front_p, use_sollinesx, use_sollinesy)
        elif use_sollinesx != [] and use_sollinesy == []:
            F_lane = supply.lane_x(front_p, use_sollinesx)
        elif use_sollinesy != [] and use_sollinesx == []:
            F_lane = supply.lane_y(front_p, use_sollinesy)
        else:  # 十字路口
            F_lane = supply.lane(front_p)

        return F_lane, use_sollinesx, use_sollinesy

    def repulsion(self, front_p, emergency, use_sollinesx, use_sollinesy):
        F_obsx, F_obsy = 0, 0

        for obstacle in self.obstacles:
            r_obs = math.sqrt((front_p[0] - obstacle[0]) ** 2 + (front_p[1] - obstacle[1]) ** 2) - 1.5 * obstacle[2]
            r_min = obstacle[3] * self.r_apf
            obs_Le = obstacle[3] * obstacle[2]
            obs_Be = 0.5 * obstacle[3] * self.B

            if obstacle[4] == 'h':
                obs_L = abs(front_p[0] - obstacle[0])
                obs_B = abs(front_p[1] - obstacle[1])
            else:
                obs_L = abs(front_p[1] - obstacle[1])
                obs_B = abs(front_p[0] - obstacle[0])

            if r_obs > r_min:
                continue
            elif r_obs <= 0:
                print("APF is failed, because the vehicle moves into the safe area.")
                exit()
            elif obs_L <= obs_Le and obs_B <= obs_Be:
                print("The vehicle meets the local minimum.")
                Fe_obsx, Fe_obsy = emergency.obstacle_force(obstacle, use_sollinesx, use_sollinesy)
                force_rep = 2 * self.k_rep * obstacle[3] * (1 / r_obs - 1 / r_min) / r_obs ** 2
                theta_rep = math.atan2(front_p[1] - obstacle[1], front_p[0] - obstacle[0])
                F_obsx += force_rep * math.cos(theta_rep) + Fe_obsx
                F_obsy += force_rep * math.sin(theta_rep) + Fe_obsy
            else:
                force_rep = 2 * self.k_rep * obstacle[3] * (1 / r_obs - 1 / r_min) / r_obs ** 2
                theta_rep = math.atan2(front_p[1] - obstacle[1], front_p[0] - obstacle[0])
                F_obsx += force_rep * math.cos(theta_rep)
                F_obsy += force_rep * math.sin(theta_rep)

        F_obs = [F_obsx, F_obsy]

        return F_obs

    def leavecar(self, front_p, moving_vehicles, emergency, use_sollinesx, use_sollinesy):
        F_vehx, F_vehy = 0, 0
        car_Le = self.L
        car_Be = 0.5 * self.B
        all_vehicles = self.vehicles + moving_vehicles

        for vehicle in all_vehicles:
            if vehicle[2] == 'h':
                car_L = abs(front_p[0] - vehicle[0])
                car_B = abs(front_p[1] - vehicle[1])
            else:
                car_L = abs(front_p[1] - vehicle[1])
                car_B = abs(front_p[0] - vehicle[0])

            if car_L > self.car_Lmin or car_B > self.car_Bmin:
                continue
            elif car_L == 0 and car_B == 0:
                print("APF is failed, because the vehicle vehicle crashes a car.")
                exit()
            elif car_L <= car_Le and car_B <= car_Be:
                print("The vehicle meets the local minimum.")
                Fe_vehx, Fe_vehy = emergency.vehicle_force(vehicle, use_sollinesx, use_sollinesy)
                if vehicle[2] == 'h':
                    F_vehx += (front_p[0] - vehicle[0]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L -
                        (front_p[1] - vehicle[1]) ** 2 / 2 / self.B) + Fe_vehx
                    F_vehy += (front_p[1] - vehicle[1]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L -
                        (front_p[1] - vehicle[1]) ** 2 / 2 / self.B) + Fe_vehy
                else:
                    F_vehx += (front_p[0] - vehicle[0]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B -
                        (front_p[1] - vehicle[1]) ** 2 / 6 / self.L) + Fe_vehx
                    F_vehy += (front_p[1] - vehicle[1]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B -
                        (front_p[1] - vehicle[1]) ** 2 / 6 / self.L) + Fe_vehy
            else:
                if vehicle[2] == 'h':
                    F_vehx += (front_p[0] - vehicle[0]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L - (front_p[1] - vehicle[1]) ** 2 / 2 / self.B)
                    F_vehy += (front_p[1] - vehicle[1]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L - (front_p[1] - vehicle[1]) ** 2 / 2 / self.B)
                else:
                    F_vehx += (front_p[0] - vehicle[0]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B - (front_p[1] - vehicle[1]) ** 2 / 6 / self.L)
                    F_vehy += (front_p[1] - vehicle[1]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B - (front_p[1] - vehicle[1]) ** 2 / 6 / self.L)

        F_veh = [F_vehx, F_vehy]

        return F_veh

    def pathplanning(self, rear_p, front_p, fi, target, iters, moving_vehicles):
        supply = SUPPLY(self.k_replane, self.k_lane, self.lw, self.h_sollines, self.v_sollines, self.t_lines)
        emergency = EMERGENCY(self.F_e, self.h_sollines, self.v_sollines, self.lw)

        distance = math.sqrt((target[0] - rear_p[0]) ** 2 + (target[1] - rear_p[1]) ** 2)

        while distance > self.target_area and iters <= self.max_iters:
            iters += 1
            print("\n----------\nNo.", iters)

            F_att = self.attraction(front_p, target)
            self.F_atts.append(F_att)
            F_lane, use_sollinesx, use_sollinesy = self.keeplane(front_p, supply)
            self.F_lanes.append(F_lane)
            F_obs = self.repulsion(front_p, emergency, use_sollinesx, use_sollinesy)
            self.F_obss.append(F_obs)
            F_veh = self.leavecar(front_p, moving_vehicles, emergency, use_sollinesx, use_sollinesy)
            self.F_vehs.append(F_veh)

            F_total = [F_att[0] + F_lane[0] + F_obs[0] + F_veh[0], F_att[1] + F_lane[1] + F_obs[1] + F_veh[1]]
            self.F_totals.append(F_total)
            F_direction = math.atan2(F_total[1], F_total[0])
            print("total force:", F_total)
            print("force direction:", int(F_direction / math.pi * 180))

            if F_direction - fi > 40 / 180 * math.pi:
                si = 40 / 180 * math.pi
            elif F_direction - fi < -40 / 180 * math.pi:
                si = -40 / 180 * math.pi
            else:
                si = F_direction - fi
            print("steering angle:", int(si / math.pi * 180))

            rear_v, rear_p, front_v, front_p, fi, vf = self.car.model(rear_p, fi, si, self.del_t)
            print("rear velocity:", rear_v, ", rear position:", rear_p)
            # print("front velocity:", front_v, ", front position:", front_p)
            # print("vehicle direction:", int(fi / math.pi * 180))

            dif_distance, dif_distance2 = self.car.difference(rear_p, front_p, si, self.del_t)
            print("difference between formula and subtract:", dif_distance - dif_distance2)
            print("distance difference:", dif_distance)

            self.car.draw_car(rear_p, front_p)
            moving_vehicles, p1, p2, p3, = self.car.draw_movingcar(moving_vehicles, self.del_t)
            # plt.pause(self.del_t)
            p1.remove()
            p2.remove()
            p3.remove()

            self.rear_velocities.append(self.car.v)
            self.path.append([rear_p[0], rear_p[1]])
            self.direction.append(fi)
            self.front_velocities.append(vf)
            self.front_points.append([front_p[0], front_p[1]])
            self.steering.append(si)

            distance = math.sqrt((target[0] - rear_p[0]) ** 2 + (target[1] - rear_p[1]) ** 2)

        if distance <= self.target_area and iters <= self.max_iters:
            print("\n----------\nAPF is successful!\n----------")
        else:
            print("\n----------\nAPF is failed!\n----------")

        return rear_p, front_p, fi, iters, moving_vehicles
