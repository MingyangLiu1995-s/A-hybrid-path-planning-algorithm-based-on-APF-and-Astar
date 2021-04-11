import math
import matplotlib.pyplot as plt


class CAR:

    def __init__(self, L, B, l, velocity):
        self.L = L
        self.B = B
        self.l = l
        self.v = velocity
        self.dif_distances = []

    def model(self, current_position, fi, si, del_t=0):
        w = self.v * math.tan(si) / self.l
        vf = self.v / math.cos(si)

        rear_v = [self.v * math.cos(fi), self.v * math.sin(fi), w]
        rear_p = [current_position[0] + del_t * rear_v[0],
                  current_position[1] + del_t * rear_v[1],
                  current_position[2] + del_t * rear_v[2]]

        current_front = [current_position[0] + self.l * math.cos(fi), current_position[1] + self.l * math.sin(fi),
                         fi + si]
        front_v = [vf * math.cos(fi + si), vf * math.sin(fi + si), w]
        front_p = [current_front[0] + del_t * front_v[0],
                   current_front[1] + del_t * front_v[1],
                   current_front[1] + del_t * front_v[2]]

        new_fi = rear_p[2]

        return rear_v, rear_p, front_v, front_p, new_fi, vf

    def difference(self, rear_p, front_p, si, del_t=0):
        dif_distance = math.sqrt(self.l ** 2 + (del_t * self.v * math.tan(si)) ** 2) - self.l
        dif_distance2 = math.sqrt((front_p[0] - rear_p[0]) ** 2 + (front_p[1] - rear_p[1]) ** 2) - self.l
        self.dif_distances.append(dif_distance)

        return dif_distance, dif_distance2

    def draw_car(self, rear_p, front_p):
        draw_x = [rear_p[0], front_p[0]]
        draw_y = [rear_p[1], front_p[1]]
        plt.plot(draw_x, draw_y, c='orange', linewidth=10)

        return

    def draw_movingcar(self, moving_vehicles, del_t=0):
        new_moving_vehicles, moving_areas, p1, p2, p3, = [], [], [], [], []

        x11 = moving_vehicles[0][0]
        y11 = moving_vehicles[0][1] - self.l / 2
        x12 = moving_vehicles[0][0]
        y12 = moving_vehicles[0][1] + self.l / 2
        draw_x1 = [x11, x12]
        draw_y1 = [y11, y12]
        p1, = plt.plot(draw_x1, draw_y1, c='green', linewidth=10)

        x21 = moving_vehicles[1][0]
        y21 = moving_vehicles[1][1] - self.l / 2
        x22 = moving_vehicles[1][0]
        y22 = moving_vehicles[1][1] + self.l / 2
        draw_x2 = [x21, x22]
        draw_y2 = [y21, y22]
        p2, = plt.plot(draw_x2, draw_y2, c='green', linewidth=10)

        x31 = moving_vehicles[2][0] - self.l / 2
        y31 = moving_vehicles[2][1]
        x32 = moving_vehicles[2][0] + self.l / 2
        y32 = moving_vehicles[2][1]
        draw_x3 = [x31, x32]
        draw_y3 = [y31, y32]
        p3, = plt.plot(draw_x3, draw_y3, c='green', linewidth=10)

        coop_x1 = moving_vehicles[0][0]
        coop_y1 = moving_vehicles[0][1] + moving_vehicles[0][3] * del_t
        coop_x2 = moving_vehicles[1][0]
        coop_y2 = moving_vehicles[1][1] + moving_vehicles[1][3] * del_t
        coop_x3 = moving_vehicles[2][0] + moving_vehicles[2][3] * del_t
        coop_y3 = moving_vehicles[2][1]
        new_moving_vehicles.append([coop_x1, coop_y1, 'v', moving_vehicles[0][3]])
        new_moving_vehicles.append([coop_x2, coop_y2, 'v', moving_vehicles[1][3]])
        new_moving_vehicles.append([coop_x3, coop_y3, 'h', moving_vehicles[2][3]])

        moving_vehicles.clear()
        moving_vehicles = new_moving_vehicles

        '''
        car_Le = 1.2 * self.L
        car_Be = 1.2 * self.B
        for vehicle in moving_vehicles:
            if vehicle[2] == 'h':
                moving_areas.append([vehicle[0] - car_Le, vehicle[1] - car_Be,
                                     vehicle[0] + car_Le, vehicle[1] + car_Be])
            else:
                moving_areas.append([vehicle[0] - car_Be, vehicle[1] - car_Le,
                                     vehicle[0] + car_Be, vehicle[1] + car_Le])
        '''

        return moving_vehicles, moving_areas, p1, p2, p3,
