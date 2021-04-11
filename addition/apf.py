import math
import numpy as np


class APF:

    def __init__(self, k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, L, B):
        self.k_att = k_att
        self.k_rep = k_rep
        self.r_apf = r_apf
        self.k_lane = k_lane
        self.k_replane = k_replane
        self.k_car = k_car
        self.lw = lane_width
        self.L = L
        self.B = B
        self.car_Lmin = 2 * L
        self.car_Bmin = B

    def attraction(self, node, target):
        distance = math.sqrt((target[0] - node[0]) ** 2 + (target[1] - node[1]) ** 2)
        Uatt = self.k_att * distance

        return Uatt

    def keeplane(self, node, solid_lines):
        yd = solid_lines[0][2]
        yu = solid_lines[1][2]

        rd = abs(node[1] - yd) - 1 / 4 * self.lw
        if rd <= 0:
            U1_d = float("inf")
        else:
            U1_d = 1 / 2 * abs(np.sign(node[1] - yd - 1 / 2 * self.lw) +
                               np.sign(node[1] - yu + 1 / 2 * self.lw)) * self.k_replane * (1 / rd - 2 / self.lw) ** 2
        ru = abs(node[1] - yu) - 1 / 4 * self.lw
        if ru <= 0:
            U1_u = float("inf")
        else:
            U1_u = 1 / 2 * abs(np.sign(node[1] - yd - 1 / 2 * self.lw) +
                               np.sign(node[1] - yu + 1 / 2 * self.lw)) * self.k_replane * (1 / ru - 2 / self.lw) ** 2
        U1 = U1_d + U1_u

        U2 = self.k_lane * self.lw * math.cos(math.pi / self.lw * ((node[1] - yd) % self.lw) + math.pi / 2)

        U3 = U1 + U2

        if U3 > 600:
            Ulane = 600
        else:
            Ulane = U3

        return Ulane

    def repulsion(self, node, obstacles):
        Uobs = 0

        for obstacle in obstacles:
            r_obs = math.sqrt((node[0] - obstacle[0]) ** 2 + (node[1] - obstacle[1]) ** 2) - 1.5 * obstacle[2]
            r_min = obstacle[3] * self.r_apf

            if r_obs > r_min:
                continue
            elif r_obs <= 0:
                if obstacle[3] == 2:
                    Uobs += 450
                else:
                    Uobs += 550
            else:
                Uobs += self.k_rep * obstacle[3] * (1 / r_obs - 1 / r_min) ** 2
                if obstacle[3] == 2:
                    if Uobs >= 450:
                        Uobs = 450
                else:
                    if Uobs >= 550:
                        Uobs = 550

        return Uobs

    def leavecar(self, node, vehicles):
        Uveh = 0

        for vehicle in vehicles:
            car_L = abs(node[0] - vehicle[0])
            car_B = abs(node[1] - vehicle[1])

            if car_L > self.car_Lmin or car_B > self.car_Bmin:
                continue
            else:
                Uveh += self.k_car * math.exp(-(node[0] - vehicle[0]) ** 2 / 6 / self.L -
                                              (node[1] - vehicle[1]) ** 2 / 2 / self.B)

        return Uveh
