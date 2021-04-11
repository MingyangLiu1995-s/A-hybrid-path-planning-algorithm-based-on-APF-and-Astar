import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from matplotlib.patches import Circle, Rectangle, Arc
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from apf import *


class MAP:

    def __init__(self, length, width, del_block):
        self.length = length
        self.width = width
        self.del_block = del_block

    def create_map(self):
        subplot = plt.figure(figsize=(5, 3))
        fig = subplot.add_subplot(111)
        fig.set_xlabel('X-distance', size=7)
        fig.set_ylabel('Y-distance', size=7)
        fig.xaxis.set_ticks_position('bottom')
        fig.yaxis.set_ticks_position('left')
        plt.tick_params(labelsize=7)
        x_major = MultipleLocator(5)
        y_major = MultipleLocator(5)
        fig.xaxis.set_major_locator(x_major)
        fig.yaxis.set_major_locator(y_major)
        plt.xlim(0, self.length)
        plt.ylim(0, self.width)

        return fig

    def draw_lines(self, solid_lines, dotted_lines):
        for solid_line in solid_lines:
            plt.plot([solid_line[0], solid_line[1]], [solid_line[2], solid_line[2]], c='black', linewidth=2)

        for dotted_line in dotted_lines:
            plt.plot([dotted_line[0], dotted_line[1]], [dotted_line[2], dotted_line[2]], c='gray', linestyle='--',
                     linewidth=2)  # xx, yy

        return

    def create_lanes(self):
        solid_lines = [[0, 50, 8], [0, 50, 22]]
        dotted_lines = [[0, 50, 11.5], [0, 50, 15], [0, 50, 18.5]]
        self.draw_lines(solid_lines, dotted_lines)

        return solid_lines, dotted_lines

    def draw_cycle(self, fig, obstacles):
        for obstacle in obstacles:
            if obstacle[3] == 2:
                cir = Circle(xy=(obstacle[0], obstacle[1]), radius=obstacle[2], facecolor='yellow')
            else:
                cir = Circle(xy=(obstacle[0], obstacle[1]), radius=obstacle[2], facecolor='red')
            fig.add_patch(cir)

        return

    def get_obstacles(self, fig):
        obstacles = [[25, 14, 2, 2], [35, 16, 2, 3]]
        self.draw_cycle(fig, obstacles)

        return obstacles

    def draw_vehicle(self, fig, vehicles, L, B):
        for vehicle in vehicles:
            x1 = vehicle[0] - L / 2
            y1 = vehicle[1] - B / 2
            rec = Rectangle(xy=(x1, y1), width=L, height=B, color='blue')
            fig.add_patch(rec)

        return

    def get_vehicles(self, fig, L, B):
        vehicles = [[13, 17]]
        self.draw_vehicle(fig, vehicles, L, B)

        return vehicles

    def create_areas(self, solid_lines):
        x1 = solid_lines[0][0]
        y1 = solid_lines[0][2]
        x2 = solid_lines[1][1]
        y2 = solid_lines[1][2]
        effective_area = [x1, y1, x2, y2]

        x3 = solid_lines[0][0]
        y3 = 0
        x4 = solid_lines[0][1]
        y4 = solid_lines[0][2]
        x5 = solid_lines[1][0]
        y5 = solid_lines[1][2]
        x6 = solid_lines[1][1]
        y6 = self.width
        infeasible_areas = [[x3, y3, x4, y4], [x5, y5, x6, y6]]

        height = solid_lines[1][2] - solid_lines[0][2]

        return effective_area, infeasible_areas, height

    def get_nodes(self, effective_area, infeasible_areas, height):
        block1 = int(self.length / self.del_block)
        block2 = int(height / self.del_block)

        nodes, node_x, node_y = [], [], []
        for i in range(block1 + 1):
            node_x.append(round(i * self.del_block, 2))
        for j in range(block2 + 1):
            node_y.append(round(effective_area[1] + j * self.del_block, 2))
        for i in range(block1 + 1):
            for j in range(block2 + 1):
                nodes.append([node_x[i], node_y[j]])

        x1 = np.linspace(infeasible_areas[0][0], infeasible_areas[0][2], 2)
        y1 = np.linspace(infeasible_areas[0][1], infeasible_areas[0][3], 2)
        X1, Y1 = np.meshgrid(x1, y1)
        Z1 = X1 * 0 + Y1 * 0
        x2 = np.linspace(infeasible_areas[1][0], infeasible_areas[1][2], 2)
        y2 = np.linspace(infeasible_areas[1][1], infeasible_areas[1][3], 2)
        X2, Y2 = np.meshgrid(x2, y2)
        Z2 = X2 * 0 + Y2 * 0
        useless_nodes = [[X1, Y1, Z1], [X2, Y2, Z2]]

        return nodes, useless_nodes

    def get_startandtarget(self):
        start = [5, 10]
        target = [45, 20]
        plt.plot(start[0], start[1], '*', color='purple')
        plt.plot(target[0], target[1], 'o', color='purple')

        return start, target

    def appear_attraction(self, nodes, useless_nodes, target, apf):
        U_att, U_x, U_y, U_z = [], [], [], []

        for node in nodes:
            Uatt = apf.attraction(node, target)
            U_att.append([node[0], node[1], Uatt])

        for an in U_att:
            U_x.append(an[0])
            U_y.append(an[1])
            U_z.append(round(an[2], 1))

        fig1 = plt.figure()
        ax = fig1.add_subplot(111, projection='3d')
        ax.plot_trisurf(U_x, U_y, U_z, cmap=cm.jet, linewidth=0.1)
        ax.plot_surface(useless_nodes[0][0], useless_nodes[0][1], useless_nodes[0][2], color='g', alpha=0.3)
        ax.plot_surface(useless_nodes[1][0], useless_nodes[1][1], useless_nodes[1][2], color='g', alpha=0.3)

        print("U_attx:", U_x)
        print("U_atty:", U_y)
        print("U_attz:", U_z)

        return U_att

    def appear_lanerepulsion(self, nodes, useless_nodes, solid_lines, apf):
        U_lane, U_x, U_y, U_z = [], [], [], []

        for node in nodes:
            Ulane = apf.keeplane(node, solid_lines)
            U_lane.append([node[0], node[1], Ulane])

        for bn in U_lane:
            if bn[2] != float("inf"):
                U_x.append(bn[0])
                U_y.append(bn[1])
                U_z.append(round(bn[2], 1))

        fig2 = plt.figure()
        bx = fig2.add_subplot(111, projection='3d')
        bx.plot_trisurf(U_x, U_y, U_z, cmap=cm.jet, linewidth=0.1)
        bx.plot_surface(useless_nodes[0][0], useless_nodes[0][1], useless_nodes[0][2], color='g', alpha=0.3)
        bx.plot_surface(useless_nodes[1][0], useless_nodes[1][1], useless_nodes[1][2], color='g', alpha=0.3)

        print("U_lanex:", U_x)
        print("U_laney:", U_y)
        print("U_lanez:", U_z)

        return U_lane

    def appear_obsrepulsion(self, nodes, useless_nodes, obstacles, apf):
        U_obs, U_x, U_y, U_z, no_nodes2 = [], [], [], [], []

        for node in nodes:
            Uobs = apf.repulsion(node, obstacles)
            U_obs.append([node[0], node[1], Uobs])

        for cn in U_obs:
            U_x.append(cn[0])
            U_y.append(cn[1])
            U_z.append(round(cn[2], 1))

        fig3 = plt.figure()
        cx = fig3.add_subplot(111, projection='3d')
        cx.plot_trisurf(U_x, U_y, U_z, cmap=cm.jet, linewidth=0.1)
        cx.plot_surface(useless_nodes[0][0], useless_nodes[0][1], useless_nodes[0][2], color='g', alpha=0.3)
        cx.plot_surface(useless_nodes[1][0], useless_nodes[1][1], useless_nodes[1][2], color='g', alpha=0.3)

        print("U_obsx:", U_x)
        print("U_obsy:", U_y)
        print("U_obsz:", U_z)

        return U_obs

    def appear_vehirepulsion(self, nodes, useless_nodes, vehicles, apf):
        U_veh, U_x, U_y, U_z = [], [], [], []

        for node in nodes:
            Uveh = apf.leavecar(node, vehicles)
            U_veh.append([node[0], node[1], Uveh])

        for dn in U_veh:
            U_x.append(dn[0])
            U_y.append(dn[1])
            U_z.append(round(dn[2], 1))

        fig4 = plt.figure()
        dx = fig4.add_subplot(111, projection='3d')
        dx.plot_trisurf(U_x, U_y, U_z, cmap=cm.jet, linewidth=0.1)
        dx.plot_surface(useless_nodes[0][0], useless_nodes[0][1], useless_nodes[0][2], color='g', alpha=0.3)
        dx.plot_surface(useless_nodes[1][0], useless_nodes[1][1], useless_nodes[1][2], color='g', alpha=0.3)

        print("U_veh U_x:", U_x)
        print("U_veh U_y:", U_y)
        print("U_veh U_z:", U_z)

        return U_veh

    def total_potential(self, nodes, useless_nodes, U_att, U_lane, U_obs, U_veh):
        U_total, U_x, U_y, U_z = [], [], [], []

        for i in range(len(nodes)):
            U_potential = U_att[i][2] + U_lane[i][2] + U_obs[i][2] + U_veh[i][2]
            U_total.append([nodes[i][0], nodes[i][1], U_potential])

        for en in U_total:
            U_x.append(en[0])
            U_y.append(en[1])
            U_z.append(round(en[2], 1))

        fig5 = plt.figure()
        ex = fig5.add_subplot(111, projection='3d')
        ex.plot_trisurf(U_x, U_y, U_z, cmap=cm.jet, linewidth=0.1)
        ex.plot_surface(useless_nodes[0][0], useless_nodes[0][1], useless_nodes[0][2], color='g', alpha=0.3)
        ex.plot_surface(useless_nodes[1][0], useless_nodes[1][1], useless_nodes[1][2], color='g', alpha=0.3)

        print("U_totalx:", U_x)
        print("U_totaly:", U_y)
        print("U_totalz:", U_z)

        return U_total
