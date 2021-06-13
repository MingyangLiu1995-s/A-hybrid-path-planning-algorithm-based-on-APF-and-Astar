import math
import matplotlib.pyplot as plt


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

    def __init__(self, del_block, astar_start, astar_target, free_nodes):
        self.del_block = del_block
        self.astar_start = astar_start
        self.astar_target = astar_target
        self.open_set = free_nodes
        self.close_set = []

    def possible_moves(self, current_node):
        next_nodes, next_costs = [], []

        possible_nodes = [[current_node[0] + self.del_block, current_node[1]],
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

        for i in range(8):
            if possible_nodes[i] in self.open_set:
                next_nodes.append(possible_nodes[i])
                next_costs.append(possible_costs[i])

        return next_nodes, next_costs

    def get_minf(self, best, next):
        if next.f < best.f:
            best = next

        return best

    def findway(self):
        path, direction = [], []
        success = 0

        father_node = self.astar_start
        current_node = self.astar_start
        current = ASTAR.NODE(current_node, father_node, self.astar_target)

        while current.current_node != self.astar_target:

            if current.current_node in self.open_set:
                self.open_set.remove(current.current_node)
                self.close_set.append(current.current_node)

                path.append(current.current_node)
                print("current node:", current.current_node, ", previous angle:", current.direction, ", current g:",
                      current.g, ", h:", current.h, ", f:", current.f)
                # plt.plot(current.current_node[0], current.current_node[1], '.k', markersize=5)
                # plt.pause(0.1)

                next_nodes, next_costs = self.possible_moves(current.current_node)
                next_set = []
                for next_node, next_cost in zip(next_nodes, next_costs):
                    next = ASTAR.NODE(next_node, current.current_node, self.astar_target, next_cost + current.g)
                    next_set.append(next)

                current.f = float("inf")
                best = current
                for next in next_set:
                    best = self.get_minf(best, next)

                current_node = best.current_node
                father_node = best.father_node
                current_cost = best.g
                current = ASTAR.NODE(current_node, father_node, self.astar_target, current_cost)
                direction.append(current.direction)

            else:
                print("A* pre path planning is failed!\n----------")

                return path, direction, success

        print("A* pre path planning is successful!\n----------")
        success = 1
        path.append(current.current_node)
        # plt.plot(current.current_node[0], current.current_node[1], '.k', markersize=5)
        # plt.pause(0.1)

        return path, direction, success

    def find_local(self, path, direction):
        local_targets = []

        path.remove(path[0])
        path.remove(path[-1])

        angle = []
        for i in range(len(direction) - 1):
            angle.append(direction[i + 1] - direction[i])

        i = 1
        for pa, ang in zip(path, angle):
            if ang == 0:
                continue
            else:
                if pa[0] == self.astar_start[0] or pa[1] == self.astar_start[1]:
                    local_targets.append(pa)
                    plt.plot(pa[0], pa[1], 'o', color='orangered', markersize=10)
                    # plt.pause(0.1)
                elif pa[0] == self.astar_target[0] or pa[1] == self.astar_target[1]:
                    continue
                else:
                    if i % 2 == 0:
                        local_targets.append(pa)
                        plt.plot(pa[0], pa[1], 'o', color='orangered', markersize=10)
                        # plt.pause(0.1)
                        i += 1
                    else:
                        i += 1

        return local_targets
