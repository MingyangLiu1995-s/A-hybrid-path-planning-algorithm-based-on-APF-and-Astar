import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from matplotlib.patches import Arc, Circle, Rectangle


class MAP:

    def __init__(self, length, width, del_block):
        self.length = length
        self.width = width
        self.del_block = del_block
        self.no_block_x = int(length / del_block)
        self.no_block_y = int(width / del_block)

    def create_map(self):
        subplot = plt.figure(figsize=(8, 8))
        fig = subplot.add_subplot(111)
        fig.set_xlabel('X-distance (m)', size=15)
        fig.set_ylabel('Y-distance (m)', size=15)
        fig.xaxis.set_ticks_position('bottom')
        fig.yaxis.set_ticks_position('left')
        plt.tick_params(labelsize=15)
        x_major = MultipleLocator(10)
        y_major = MultipleLocator(10)
        fig.xaxis.set_major_locator(x_major)
        fig.yaxis.set_major_locator(y_major)
        plt.xlim(0, self.length)
        plt.ylim(0, self.width)

        return fig

    def draw_lines(self, fig, solid_lines, dotted_lines, transition_lines):
        for solid_line in solid_lines:
            if solid_line[3] == 'h':
                plt.plot([solid_line[0], solid_line[1]], [solid_line[2], solid_line[2]], c='black', linewidth=2)
                # xx, yy
            else:
                plt.plot([solid_line[0], solid_line[0]], [solid_line[1], solid_line[2]], c='black', linewidth=2)

        for dotted_line in dotted_lines:
            if dotted_line[3] == 'h':
                plt.plot([dotted_line[0], dotted_line[1]], [dotted_line[2], dotted_line[2]], c='gray', linestyle='--',
                         linewidth=2)
            else:
                plt.plot([dotted_line[0], dotted_line[0]], [dotted_line[1], dotted_line[2]], c='gray', linestyle='--',
                         linewidth=2)

        arc1 = Arc(xy=(transition_lines[0][0], transition_lines[0][1]), width=2*transition_lines[0][2],
                   height=2*transition_lines[0][2], angle=90, theta1=-180, theta2=-90, color='black', linewidth=2)
        fig.add_patch(arc1)
        arc2 = Arc(xy=(transition_lines[1][0], transition_lines[1][1]), width=2*transition_lines[1][2],
                   height=2*transition_lines[1][2], angle=90, theta1=-90, theta2=0, color='black', linewidth=2)
        fig.add_patch(arc2)
        arc3 = Arc(xy=(transition_lines[2][0], transition_lines[2][1]), width=2*transition_lines[2][2],
                   height=2*transition_lines[2][2], angle=90, theta1=90, theta2=180, color='black', linewidth=2)
        fig.add_patch(arc3)
        arc4 = Arc(xy=(transition_lines[3][0], transition_lines[3][1]), width=2*transition_lines[3][2],
                   height=2*transition_lines[3][2], angle=90, theta1=0, theta2=90, color='black', linewidth=2)
        fig.add_patch(arc4)

        return

    def create_lanes(self, fig):
        solid_lines = [[0, 80, 0, 'h'], [0, 47.5, 14, 'h'], [66.5, 80, 14, 'h'], [50, 16.5, 70.5, 'v'],
                       [64, 16.5, 70.5, 'v'], [0, 47.5, 73, 'h'], [66.5, 80, 73, 'h'], [0, 80, 80, 'h']]
        dotted_lines = [[0, 50, 3.5, 'h'], [0, 50, 7, 'h'], [0, 50, 10.5, 'h'], [64, 80, 3.5, 'h'],
                        [64, 80, 7, 'h'], [64, 80, 10.5, 'h'], [53.5, 14, 73, 'v'], [57, 14, 73, 'v'],
                        [60.5, 14, 73, 'v'], [0, 50, 76.5, 'h'], [64, 80, 76.5, 'h']]
        transition_lines = [[47.5, 16.5, 2.5], [47.5, 70.5, 2.5], [66.5, 16.5, 2.5], [66.5, 70.5, 2.5]]

        self.draw_lines(fig, solid_lines, dotted_lines, transition_lines)

        return solid_lines, dotted_lines, transition_lines

    def draw_cycle(self, fig, obstacles):
        for obstacle in obstacles:
            if obstacle[3] == 2:
                cir = Circle(xy=(obstacle[0], obstacle[1]), radius=obstacle[2], facecolor='yellow')
            else:
                cir = Circle(xy=(obstacle[0], obstacle[1]), radius=obstacle[2], facecolor='red')
            fig.add_patch(cir)

        return

    def get_obstacles(self, fig):
        # obstacles = [[17, 1, 1.5, 2, 'h'], [30, 4, 2, 3, 'h'], [50.5, 28, 2, 2, 'v'], [64, 35, 1.5, 2, 'v']]
        obstacles = [[17, 1, 1.5, 2, 'h'], [30, 4, 2, 3, 'h'], [50.5, 28, 2, 2, 'v']]
        self.draw_cycle(fig, obstacles)

        return obstacles

    def draw_vehicle(self, fig, vehicles, L, B):
        for vehicle in vehicles:
            if vehicle[2] == 'h':
                x1 = vehicle[0] - L / 2
                y1 = vehicle[1] - B / 2
                rec = Rectangle(xy=(x1, y1), width=L, height=B, color='blue')
                fig.add_patch(rec)
            else:
                x1 = vehicle[0] - B / 2
                y1 = vehicle[1] - L / 2
                rec = Rectangle(xy=(x1, y1), width=B, height=L, color='blue')
                fig.add_patch(rec)

        return

    def draw_initmovingvehi(self, fig, init_movingvehis, L, B):
        for moving_vehicle in init_movingvehis:
            if moving_vehicle[2] == 'h':
                x1 = moving_vehicle[0] - L / 2
                y1 = moving_vehicle[1] - B / 2
                rec = Rectangle(xy=(x1, y1), width=L, height=B, color='green')
                fig.add_patch(rec)
            else:
                x1 = moving_vehicle[0] - B / 2
                y1 = moving_vehicle[1] - L / 2
                rec = Rectangle(xy=(x1, y1), width=B, height=L, color='green')
                fig.add_patch(rec)

        return

    def get_vehicles(self, fig, L, B):
        # vehicles = [[45, 8.5, 'h'], [5, 5.5, 'h'], [55, 40, 'v'], [58.5, 50, 'v'], [18, 12, 'h']]
        # init_movingvehis = [[10, 85, 'h', 3], [10, 88, 'h', 3], [10, 90, 'h', 3]]
        # moving_vehicles = [[10, 85, 'h', 3], [10, 88, 'h', 3], [10, 90, 'h', 3]]

        vehicles = [[45, 8.5, 'h'], [5, 5.5, 'h']]
        init_movingvehis = [[55, 55, 'v', -3], [58.5, 50, 'v', 3], [15, 12, 'h', 3]]
        moving_vehicles = [[55, 55, 'v', -3], [58.5, 50, 'v', 3], [15, 12, 'h', 3]]

        self.draw_vehicle(fig, vehicles, L, B)
        # self.draw_initmovingvehi(fig, init_movingvehis, L, B)

        return vehicles, init_movingvehis, moving_vehicles

    def create_areas(self):
        infeasible_areas = [[0, 14, 50, 73], [64, 14, 80, 73]]

        return infeasible_areas

    def get_nodes(self, infeasible_areas, solid_lines):
        nodes, free_nodes, new_x, new_y = [], [], [], []

        for i in range(self.no_block_x + 1):
            new_x.append(int(i * self.del_block))
            # plt.axvline(x=new_x[i], color='lightgray', lw=0.5)

        for j in range(self.no_block_y + 1):
            new_y.append(int(j * self.del_block))
            # plt.axhline(y=new_y[j], color='lightgray', lw=0.5)

        for i in range(self.no_block_x + 1):
            for j in range(self.no_block_y + 1):
                nodes.append([new_x[i], new_y[j]])
                free_nodes.append([new_x[i], new_y[j]])

        for area in infeasible_areas:
            for node in nodes:
                if area[0] <= node[0] <= area[2] and area[1] <= node[1] <= area[3]:
                    if node in free_nodes:
                        free_nodes.remove([node[0], node[1]])

        for solid_line in solid_lines:
            for node in nodes:
                if solid_line[3] == "h":
                    if solid_line[0] <= node[0] <= solid_line[1] and node[1] == solid_line[2]:
                        if node in free_nodes:
                            free_nodes.remove([node[0], node[1]])
                else:
                    if node[0] == solid_line[0] and solid_line[1] <= node[1] <= solid_line[2]:
                        if node in free_nodes:
                            free_nodes.remove([node[0], node[1]])

        '''
        for free_node in free_nodes:
            plt.plot(free_node[0], free_node[1], 'x', color='lightblue')
        '''

        return nodes, free_nodes

    def get_startandtarget(self):
        # start = [5, 2]
        # target = [63, 60]
        start = [5, 2]
        target = [78, 78]

        plt.plot(start[0], start[1], '*', color='purple', markersize=10)
        plt.plot(target[0], target[1], 'o', color='purple', markersize=10)

        return start, target

    def final_draw(self, fig, front_points, path, init_movingvehis, moving_vehicles, L, B):
        front_x, front_y, path_x, path_y = [], [], [], []
        for i in front_points:
            front_x.append(i[0])
            front_y.append(i[1])
            plt.plot(front_x, front_y, 'o', c='green', markersize=2)
        for j in path:
            path_x.append(j[0])
            path_y.append(j[1])
        # plt.plot(front_x, front_y, c='green', linewidth=2)
        plt.plot(path_x, path_y, c='r', linewidth=2)

        for v1, v2 in zip(init_movingvehis, moving_vehicles):
            if v1[2] == 'h':
                x1 = v1[0] - L / 2
                y1 = v1[1] - B / 2
                h = v2[0] + L / 2 - x1
                rec = Rectangle(xy=(x1, y1), width=h, height=B, color='green', alpha=0.3)
                fig.add_patch(rec)
            else:
                x1 = v1[0] - B / 2
                y1 = v1[1] - L / 2
                h = v2[1] + L / 2 - y1
                rec = Rectangle(xy=(x1, y1), width=B, height=h, color='green', alpha=0.3)
                fig.add_patch(rec)

        for vehicle in moving_vehicles:
            if vehicle[2] == 'h':
                x1 = vehicle[0] - L / 2
                y1 = vehicle[1] - B / 2
                rec = Rectangle(xy=(x1, y1), width=L, height=B, color='green')
                fig.add_patch(rec)
            else:
                x1 = vehicle[0] - B / 2
                y1 = vehicle[1] - L / 2
                rec = Rectangle(xy=(x1, y1), width=B, height=L, color='green')
                fig.add_patch(rec)

        # plt.pause(0.1)

        return
