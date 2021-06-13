import math
import numpy as np
import matplotlib.pyplot as plt


class SUPPLY:

    def __init__(self, k_replane, k_lane, lw, h_sollines, v_sollines, t_lines):
        self.k_replane = k_replane
        self.k_lane = k_lane
        self.lw = lw
        self.h_sollines = h_sollines
        self.v_sollines = v_sollines
        self.t_lines = t_lines

    def useful_lines(self, front_p):
        use_sollinesx, use_sollinesy = [], []

        for solid_line in self.h_sollines:
            xl, xr, y = solid_line[0], solid_line[1], solid_line[2]
            if xl <= front_p[0] <= xr:
                if front_p[1] - 4 * self.lw <= y <= front_p[1] + 4 * self.lw:  # 取决于地图怎么画的
                    use_sollinesx.append(solid_line)

        for solid_line in self.v_sollines:
            x, yd, yu = solid_line[0], solid_line[1], solid_line[2]
            if yd <= front_p[1] <= yu:
                if front_p[0] - 4 * self.lw <= x <= front_p[0] + 4 * self.lw:
                    use_sollinesy.append(solid_line)

        return use_sollinesx, use_sollinesy

    def lane_xy(self, front_p, use_sollinesx, use_sollinesy, iters):
        yd, yu = 0, float("inf")

        for solid_line in use_sollinesx:
            if solid_line[2] < front_p[1]:
                yd = solid_line[2]
            else:
                yu = solid_line[2]

        rd = abs(front_p[1] - yd) - 1 / 4 * self.lw
        fd = abs(np.sign(front_p[1] - yd - 1 / 2 * self.lw) +
                 np.sign(front_p[1] - yu + 1 / 2 * self.lw)) * self.k_replane * (1 / rd - 2 / self.lw) / rd ** 2
        ru = abs(front_p[1] - yu) - 1 / 4 * self.lw
        fu = abs(np.sign(front_p[1] - yd - 1 / 2 * self.lw) +
                 np.sign(front_p[1] - yu + 1 / 2 * self.lw)) * self.k_replane * (1 / ru - 2 / self.lw) / ru ** 2
        force_solidy = fd + fu

        if front_p[1] >= yu - self.lw:
            F_solidy = [0, -force_solidy]
        else:
            F_solidy = [0, force_solidy]

        xl, xr = 0, float("inf")

        for solid_line in use_sollinesy:
            if solid_line[0] < front_p[0]:
                xl = solid_line[0]
            else:
                xr = solid_line[0]

        rl = abs(front_p[0] - xl) - 1 / 4 * self.lw
        fl = abs(np.sign(front_p[0] - xl - 1 / 2 * self.lw) +
                 np.sign(front_p[0] - xr + 1 / 2 * self.lw)) * self.k_replane * (1 / rl - 2 / self.lw) / rl ** 2
        rr = abs(front_p[0] - xr) - 1 / 4 * self.lw
        fr = abs(np.sign(front_p[0] - xl - 1 / 2 * self.lw) +
                 np.sign(front_p[0] - xr + 1 / 2 * self.lw)) * self.k_replane * (1 / rr - 2 / self.lw) / rr ** 2
        force_solidx = fl + fr

        if front_p[0] >= xr - self.lw:
            F_solidx = [-force_solidx, 0]
        else:
            F_solidx = [force_solidx, 0]

        F_solidtx, F_solidty = 0, 0

        for t_line in self.t_lines:
            rt = math.sqrt((front_p[0] - t_line[0]) ** 2 + (front_p[1] - t_line[1]) ** 2) - t_line[2] - 1 / 4 * self.lw

            if rt > 1 / 2 * self.lw:
                continue
            elif rt <= 0:
                print("APF is failed, because the vehicle moves into the safe distance.")
                plt.savefig("E:/pythonpro/modifyandtry/9_dongtai/results/failed" + str(iters) + ".png")
                # exit()
                break
            else:
                force_solidt = 2 * self.k_replane * (1 / rt - 2 / self.lw) / rt ** 2
                theta_solidt = math.atan2(front_p[1] - t_line[1], front_p[0] - t_line[0])
                F_solidtx += force_solidt * math.cos(theta_solidt)
                F_solidty += force_solidt * math.sin(theta_solidt)

        F_solidt = [F_solidtx, F_solidty]

        F_solid = [F_solidx[0] + F_solidy[0] + F_solidt[0], F_solidx[1] + F_solidy[1] + F_solidt[1]]
        F_dotted = [0, 0]
        F_lane = [F_solid[0] + F_dotted[0], F_solid[1] + F_dotted[1]]

        return F_lane

    def lane_x(self, front_p, use_sollinesx, iters):
        yd, yu = 0, float("inf")

        for solid_line in use_sollinesx:
            if solid_line[2] < front_p[1]:
                yd = solid_line[2]
            else:
                yu = solid_line[2]

        rd = abs(front_p[1] - yd) - 1 / 4 * self.lw
        fd = abs(np.sign(front_p[1] - yd - 1 / 2 * self.lw) +
                 np.sign(front_p[1] - yu + 1 / 2 * self.lw)) * self.k_replane * (1 / rd - 2 / self.lw) / rd ** 2
        ru = abs(front_p[1] - yu) - 1 / 4 * self.lw
        fu = abs(np.sign(front_p[1] - yd - 1 / 2 * self.lw) +
                 np.sign(front_p[1] - yu + 1 / 2 * self.lw)) * self.k_replane * (1 / ru - 2 / self.lw) / ru ** 2
        force_solidy = fd + fu

        if front_p[1] >= yu - self.lw:
            F_solidy = [0, -force_solidy]
        else:
            F_solidy = [0, force_solidy]

        F_solidtx, F_solidty = 0, 0

        for t_line in self.t_lines:
            rt = math.sqrt((front_p[0] - t_line[0]) ** 2 + (front_p[1] - t_line[1]) ** 2) - t_line[2] - 1 / 4 * self.lw

            if rt > 1 / 2 * self.lw:
                continue
            elif rt <= 0:
                print("APF is failed, because the vehicle moves into the safe distance.")
                plt.savefig("E:/pythonpro/modifyandtry/9_dongtai/results/failed" + str(iters) + ".png")
                # exit()
                break
            else:
                force_solidt = 2 * self.k_replane * (1 / rt - 2 / self.lw) / rt ** 2
                theta_solidt = math.atan2(front_p[1] - t_line[1], front_p[0] - t_line[0])
                F_solidtx += force_solidt * math.cos(theta_solidt)
                F_solidty += force_solidt * math.sin(theta_solidt)

        F_solidt = [F_solidtx, F_solidty]

        force_dotted = self.k_lane * math.pi * math.sin(math.pi / self.lw * ((front_p[1] - yd) % self.lw) + math.pi / 2)
        F_dottedy = [0, force_dotted]

        F_solid = [F_solidy[0] + F_solidt[0], F_solidy[1] + F_solidt[1]]
        F_dotted = F_dottedy
        F_lane = [F_solid[0] + F_dotted[0], F_solid[1] + F_dotted[1]]

        return F_lane

    def lane_y(self, front_p, use_sollinesy, iters):
        xl, xr = 0, float("inf")

        for solid_line in use_sollinesy:
            if solid_line[0] < front_p[0]:
                xl = solid_line[0]
            else:
                xr = solid_line[0]

        rl = abs(front_p[0] - xl) - 1 / 4 * self.lw
        fl = abs(np.sign(front_p[0] - xl - 1 / 2 * self.lw) +
                 np.sign(front_p[0] - xr + 1 / 2 * self.lw)) * self.k_replane * (1 / rl - 2 / self.lw) / rl ** 2
        rr = abs(front_p[0] - xr) - 1 / 4 * self.lw
        fr = abs(np.sign(front_p[0] - xl - 1 / 2 * self.lw) +
                 np.sign(front_p[0] - xr + 1 / 2 * self.lw)) * self.k_replane * (1 / rr - 2 / self.lw) / rr ** 2
        force_solidx = fl + fr

        if front_p[0] >= xr - self.lw:
            F_solidx = [-force_solidx, 0]
        else:
            F_solidx = [force_solidx, 0]

        F_solidtx, F_solidty = 0, 0

        for t_line in self.t_lines:
            rt = math.sqrt((front_p[0] - t_line[0]) ** 2 + (front_p[1] - t_line[1]) ** 2) - t_line[2] - 1 / 4 * self.lw

            if rt > 1 / 2 * self.lw:
                continue
            elif rt <= 0:
                print("APF is failed, because the vehicle moves into the safe distance.")
                plt.savefig("E:/pythonpro/modifyandtry/9_dongtai/results/" + str(iters) + "failed.png")
                # exit()
                break
            else:
                force_solidt = 2 * self.k_replane * (1 / rt - 2 / self.lw) / rt ** 2
                theta_solidt = math.atan2(front_p[1] - t_line[1], front_p[0] - t_line[0])
                F_solidtx += force_solidt * math.cos(theta_solidt)
                F_solidty += force_solidt * math.sin(theta_solidt)

        F_solidt = [F_solidtx, F_solidty]

        force_dotted = self.k_lane * math.pi * math.sin(math.pi / self.lw * ((front_p[0] - xl) % self.lw) + math.pi / 2)
        F_dottedx = [force_dotted, 0]

        F_solid = [F_solidx[0] + F_solidt[0], F_solidx[1] + F_solidt[1]]
        F_dotted = F_dottedx
        F_lane = [F_solid[0] + F_dotted[0], F_solid[1] + F_dotted[1]]

        return F_lane

    def lane(self, front_p, iters):
        F_solidtx, F_solidty = 0, 0

        for t_line in self.t_lines:
            rt = math.sqrt((front_p[0] - t_line[0]) ** 2 + (front_p[1] - t_line[1]) ** 2) - t_line[2] - 1 / 4 * self.lw

            if rt > 1 / 2 * self.lw:
                continue
            elif rt <= 0:
                print("APF is failed, because the vehicle moves into the safe distance.")
                plt.savefig("E:/pythonpro/modifyandtry/9_dongtai/results/failed" + str(iters) + ".png")
                # exit()
                break
            else:
                force_solidt = 2 * self.k_replane * (1 / rt - 2 / self.lw) / rt ** 2
                theta_solidt = math.atan2(front_p[1] - t_line[1], front_p[0] - t_line[0])
                F_solidtx += force_solidt * math.cos(theta_solidt)

        F_solidt = [F_solidtx, F_solidty]

        F_solid = F_solidt
        F_dotted = [0, 0]
        F_lane = [F_solid[0] + F_dotted[0], F_solid[1] + F_dotted[1]]

        return F_lane


class EMERGENCY:

    def __init__(self, F_e, h_sollines, v_sollines, lw):
        self.F_e = F_e
        self.h_sollines = h_sollines
        self.v_sollines = v_sollines
        self.lw = lw

    def obstacle_force(self, obstacle, use_sollinesx, use_sollinesy):
        Fe_obsx, Fe_obsy = 0, 0

        if obstacle[4] == 'h':
            yd, yu = 0, float("inf")
            for solid_line in use_sollinesx:
                if solid_line[2] < obstacle[1]:
                    yd = solid_line[2]
                else:
                    yu = solid_line[2]

            if yu - obstacle[1] < obstacle[1] - yd:
                Fe_obsy = -self.F_e
                print("Fe:", Fe_obsy)
            else:
                Fe_obsy = self.F_e
                print("Fe:", Fe_obsy)

        else:
            xl, xr = 0, float("inf")
            for solid_line in use_sollinesy:
                if solid_line[0] < obstacle[0]:
                    xl = solid_line[0]
                else:
                    xr = solid_line[0]

            if xr - obstacle[0] < obstacle[0] - xl:
                Fe_obsx = -self.F_e
                print("Fe:", Fe_obsx)
            else:
                Fe_obsx = self.F_e
                print("Fe:", Fe_obsx)

        return Fe_obsx, Fe_obsy

    def vehicle_force(self, vehicle, use_sollinesx, use_sollinesy):
        Fe_vehx, Fe_vehy = 0, 0

        if vehicle[2] == 'h':
            yd, yu = 0, float("inf")
            for solid_line in use_sollinesx:
                if solid_line[2] < vehicle[1]:
                    yd = solid_line[2]
                else:
                    yu = solid_line[2]

            if yu - vehicle[1] < vehicle[1] - yd:
                Fe_vehy = -self.F_e
                print("Fe:", Fe_vehy)
            else:
                Fe_vehy = self.F_e
                print("Fe:", Fe_vehy)

        else:
            xl, xr = 0, float("inf")
            for solid_line in use_sollinesy:
                if solid_line[0] < vehicle[0]:
                    xl = solid_line[0]
                else:
                    xr = solid_line[0]

            if xr - vehicle[0] < vehicle[0] - xl:
                Fe_vehx = -self.F_e
                print("Fe:", Fe_vehx)
            else:
                Fe_vehx = self.F_e
                print("Fe:", Fe_vehx)

        return Fe_vehx, Fe_vehy
