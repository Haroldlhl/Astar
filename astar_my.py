# 定义一个节点类，存储节点的坐标和各种代价
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.r = 0.0
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0
        self.neighbors = []
        self.previous = None
        # GRID_STATE_SET = {'safe': 0, 'obstacle': 1, 'open': 2, 'close': 3,
        #                  'goal': 4, 'start': 5, 'to be filled': 7, 'near': 8}
        self.sort = 'safe'

    # 定义节点之间的比较方式
    def __lt__(self, other):
        return self.f < other.f

    def add_neighbors(self, grid):

        neighbor_x = self.x
        neighbor_y = self.y

        if neighbor_x < MAP_HEIGHT - 1:
            print((neighbor_x+1, neighbor_y))
            self.neighbors.append(grid[neighbor_x + 1][neighbor_y])
        if neighbor_x > 0:
            self.neighbors.append(grid[neighbor_x - 1][neighbor_y])
        if neighbor_y < MAP_WIDTH - 1:
            self.neighbors.append(grid[neighbor_x][neighbor_y + 1])
        if neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x][neighbor_y - 1])

        if neighbor_x > 0 and neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x-1][neighbor_y-1])
        if neighbor_x < MAP_HEIGHT-1 and neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x+1][neighbor_y-1])
        if neighbor_x > 0 and neighbor_y < MAP_WIDTH-1:
            self.neighbors.append(grid[neighbor_x-1][neighbor_y+1])
        if neighbor_x < MAP_HEIGHT-1 and neighbor_y < MAP_WIDTH-1:
            self.neighbors.append(grid[neighbor_x+1][neighbor_y+1])

class AStar:

    def __init__(self, start, end):
        self.start = start
        self.end = end

    @staticmethod
    def h_score(current_node, end):

        dx = abs(current_node.x - end.x)
        dy = abs(current_node.y - end.y)
        # 棋盘格距离
        # # dx 总是较大的值
        # if dx < dy:
        #     dx, dy = dy, dx
        # distance = 14*dy + 10*(dx-dy)

        # # 欧式距离, 效果还不如棋盘格（切比雪夫）
        # 曼哈顿距离
        # distance = 10*max(dx, dy)
        # distance = np.floor(20*np.sqrt(dx**2 + dy**2))

        # 为减少转弯，增添转弯代价
        # p_node = current_node.previous
        # pp_node = p_node.previous
        # if current_node.x + pp_node.x != 2 * p_node.x or current_node.y + pp_node.y != 2 * p_node.y:
        #     distance += 30'
        # distance = np.sqrt(dx**2 + dy**2)
        distance = max(dx, dy)

        return distance

    @staticmethod
    def create_nodes(world_map):

        global map_colors
        nodes = []
        for i in range(MAP_HEIGHT):
            nodes.append([])
            for j in range(MAP_WIDTH):
                node = Node(i, j)
                nodes[-1].append(node)

                if world_map[i][j] == 0:
                    node.sort = 'obstacle'
                    map_colors[i][j] = 1
                if world_map[i][j] == 255:
                    node.sort = 'safe'
                    map_colors[i][j] = 0

        # 对于集群和目标节点进行修改
        nodes[START_LOC[0]][START_LOC[1]].sort = 'start'
        nodes[END_LOC[0]][END_LOC[1]].sort = 'goal'

        return nodes

    @staticmethod
    def get_neighbors(grid):
        for i in range(MAP_HEIGHT):
            for j in range(MAP_WIDTH):
                grid[i][j].add_neighbors(grid)
        return grid

    @staticmethod
    def update_sets_once(open_set, closed_set, end, nodes):
        """A*算法最主要的执行部分, 刷新一次集合，计算一步

        :param open_set: 开集
        :param closed_set: 闭集
        :param end: 终点
        :return: 新的开集，闭集，当前节点位置，最终路径（未找到则为空）（倒叙）
        """

        # 找到在开集中，值最小的点
        best_way = 0
        for i, node in enumerate(open_set):
            if node.f < open_set[best_way].f:
                best_way = i

        current_node = open_set[best_way]
        open_set.pop(best_way)
        final_path = []

        # 判断是否结束, 结束则溯源找到起点，生成列表
        if current_node == end:
            final_path.append(end)
            temp = current_node
            while temp.previous:
                final_path.append(temp.previous)
                temp = temp.previous

            print("Done !!")
            return [], [], [], final_path

        # 未结束的处理
        closed_set.append(current_node)
        current_node.sort = 'close'
        current_node.add_neighbors(nodes)
        neighbors = current_node.neighbors
        for neighbor in neighbors:
            if (neighbor in closed_set) or neighbor.sort == 'obstacle':
                continue    # 对于不在闭集，也不是障碍物的进行处理
            else:
                # 求欧式距离
                dist_man = abs(neighbor.x-current_node.x) + abs(neighbor.y-current_node.y)
                if dist_man == 1:
                    temp_g = current_node.g + 1
                elif dist_man == 2:
                    temp_g = current_node.g + 1
                else:
                    raise ValueError("distance invalid: the manhattan distance between the current node and its "
                                     "neighbour is {}".format(dist_man))

                # ----------分割------
                # 判断是否要更新已经在开集中的点的previous
                control_flag = False    # 标志，该邻居是否在开集中，在开集中需要对代价刷新(或者不变）， 不在开集要计算代价
                for k, node in enumerate(open_set):
                    # 如果当前节点的邻接节点已近在开集中，要判断与刷新该节点的代价值
                    if neighbor.x == node.x and neighbor.y == node.y:
                        if temp_g < node.g:
                            node.g = temp_g
                            node.previous = current_node
                            node.h = AStar.h_score(node, end)
                            node.f = node.g + node.h + node.r

                        else:
                            pass
                        control_flag = True

                if control_flag:
                    # 在开集中，节点代价计算已完成
                    pass
                else:
                    neighbor.g = temp_g
                    neighbor.previous = current_node
                    neighbor.h = AStar.h_score(neighbor, end)
                    neighbor.f = neighbor.g + neighbor.h + neighbor.r
                    neighbor.sort = 'open'
                    open_set.append(neighbor)

                    global closes
                    closes.append(neighbor)

        return open_set, closed_set, current_node, final_path

# 定义一个A星类，内部存放静态方法
def run_astar():
    """  主调用，返回的是最终路径 final_path（顺序）, 包含绘图部分

    :return:  [[x,y], [x,y]]
    """
    global map_colors

    nodes = AStar.create_nodes(map_grid)
    # TODO 可优化，不必都求neighbor
    open_set = []
    closed_set = []
    open_set.append(nodes[START_LOC[0]][START_LOC[1]])
    end_dot = nodes[END_LOC[0]][END_LOC[1]]

    # 绘图初始化
    # 创建一张RGB模式的图片，所有像素初始化为白色
    # 图片中横着的为x
    img = Image.new('RGB', (MAP_WIDTH, MAP_HEIGHT), color='white')
    rands = np.random.rand(500, 800)*0.01
    for x in range(MAP_HEIGHT):
        for y in range(MAP_WIDTH):
            node = nodes[x][y]
            node.r = rands[x][y# 利用开题报告中的ppt图片，进行栅格化后得到0.1*0.1比例的栅格二值图片
# 进行A*初步循迹，测试
import numpy as np
from PIL import Image

# 打开原始二值图像
img = Image.open("binary.png")

# 生成图像矩阵
map_grid = np.array(img)
np.save('/npy/matrix.npy', map_grid)

START_LOC, END_LOC = [24, 33], [460, 127]
map_colors = np.zeros(np.shape(map_grid), dtype=np.int8)    # 实际颜色地图，颜色取决于映射

# 定义地图的宽度和高度
MAP_HEIGHT, MAP_WIDTH = np.shape(map_grid)
cnt = 0
closes = []         # 闭集中的点


# 定义一个节点类，存储节点的坐标和各种代价
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.r = 0.0
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0
        self.neighbors = []
        self.previous = None
        # GRID_STATE_SET = {'safe': 0, 'obstacle': 1, 'open': 2, 'close': 3,
        #                  'goal': 4, 'start': 5, 'to be filled': 7, 'near': 8}
        self.sort = 'safe'

    # 定义节点之间的比较方式
    def __lt__(self, other):
        return self.f < other.f

    def add_neighbors(self, grid):

        neighbor_x = self.x
        neighbor_y = self.y

        if neighbor_x < MAP_HEIGHT - 1:
            print((neighbor_x+1, neighbor_y))
            self.neighbors.append(grid[neighbor_x + 1][neighbor_y])
        if neighbor_x > 0:
            self.neighbors.append(grid[neighbor_x - 1][neighbor_y])
        if neighbor_y < MAP_WIDTH - 1:
            self.neighbors.append(grid[neighbor_x][neighbor_y + 1])
        if neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x][neighbor_y - 1])

        if neighbor_x > 0 and neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x-1][neighbor_y-1])
        if neighbor_x < MAP_HEIGHT-1 and neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x+1][neighbor_y-1])
        if neighbor_x > 0 and neighbor_y < MAP_WIDTH-1:
            self.neighbors.append(grid[neighbor_x-1][neighbor_y+1])
        if neighbor_x < MAP_HEIGHT-1 and neighbor_y < MAP_WIDTH-1:
            self.neighbors.append(grid[neighbor_x+1][neighbor_y+1])

class AStar:

    def __init__(self, start, end):
        self.start = start
        self.end = end

    @staticmethod
    def h_score(current_node, end):

        dx = abs(current_node.x - end.x)
        dy = abs(current_node.y - end.y)
        # 棋盘格距离
        # # dx 总是较大的值
        # if dx < dy:
        #     dx, dy = dy, dx
        # distance = 14*dy + 10*(dx-dy)

        # # 欧式距离, 效果还不如棋盘格（切比雪夫）
        # 曼哈顿距离
        # distance = 10*max(dx, dy)
        # distance = np.floor(20*np.sqrt(dx**2 + dy**2))

        # 为减少转弯，增添转弯代价
        # p_node = current_node.previous
        # pp_node = p_node.previous
        # if current_node.x + pp_node.x != 2 * p_node.x or current_node.y + pp_node.y != 2 * p_node.y:
        #     distance += 30'
        # distance = np.sqrt(dx**2 + dy**2)
        distance = max(dx, dy)

        return distance

    @staticmethod
    def create_nodes(world_map):

        global map_colors
        nodes = []
        for i in range(MAP_HEIGHT):
            nodes.append([])
            for j in range(MAP_WIDTH):
                node = Node(i, j)
                nodes[-1].append(node)

                if world_map[i][j] == 0:
                    node.sort = 'obstacle'
                    map_colors[i][j] = 1
                if world_map[i][j] == 255:
                    node.sort = 'safe'
                    map_colors[i][j] = 0

        # 对于集群和目标节点进行修改
        nodes[START_LOC[0]][START_LOC[1]].sort = 'start'
        nodes[END_LOC[0]][END_LOC[1]].sort = 'goal'

        return nodes

    @staticmethod
    def get_neighbors(grid):
        for i in range(MAP_HEIGHT):
            for j in range(MAP_WIDTH):
                grid[i][j].add_neighbors(grid)
        return grid

    @staticmethod
    def update_sets_once(open_set, closed_set, end, nodes):
        """A*算法最主要的执行部分, 刷新一次集合，计算一步

        :param open_set: 开集
        :param closed_set: 闭集
        :param end: 终点
        :return: 新的开集，闭集，当前节点位置，最终路径（未找到则为空）（倒叙）
        """

        # 找到在开集中，值最小的点
        best_way = 0
        for i, node in enumerate(open_set):
            if node.f < open_set[best_way].f:
                best_way = i

        current_node = open_set[best_way]
        open_set.pop(best_way)
        final_path = []

        # 判断是否结束, 结束则溯源找到起点，生成列表
        if current_node == end:
            final_path.append(end)
            temp = current_node
            while temp.previous:
                final_path.append(temp.previous)
                temp = temp.previous

            print("Done !!")
            return [], [], [], final_path

        # 未结束的处理
        closed_set.append(current_node)
        current_node.sort = 'close'
        current_node.add_neighbors(nodes)
        neighbors = current_node.neighbors
        for neighbor in neighbors:
            if (neighbor in closed_set) or neighbor.sort == 'obstacle':
                continue    # 对于不在闭集，也不是障碍物的进行处理
            else:
                # 求欧式距离
                dist_man = abs(neighbor.x-current_node.x) + abs(neighbor.y-current_node.y)
                if dist_man == 1:
                    temp_g = current_node.g + 1
                elif dist_man == 2:
                    temp_g = current_node.g + 1
                else:
                    raise ValueError("distance invalid: the manhattan distance between the current node and its "
                                     "neighbour is {}".format(dist_man))

                # ----------分割------
                # 判断是否要更新已经在开集中的点的previous
                control_flag = False    # 标志，该邻居是否在开集中，在开集中需要对代价刷新(或者不变）， 不在开集要计算代价
                for k, node in enumerate(open_set):
                    # 如果当前节点的邻接节点已近在开集中，要判断与刷新该节点的代价值
                    if neighbor.x == node.x and neighbor.y == node.y:
                        if temp_g < node.g:
                            node.g = temp_g
                            node.previous = current_node
                            node.h = AStar.h_score(node, end)
                            node.f = node.g + node.h + node.r

                        else:
                            pass
                        control_flag = True

                if control_flag:
                    # 在开集中，节点代价计算已完成
                    pass
                else:
                    neighbor.g = temp_g
                    neighbor.previous = current_node
                    neighbor.h = AStar.h_score(neighbor, end)
                    neighbor.f = neighbor.g + neighbor.h + neighbor.r
                    neighbor.sort = 'open'
                    open_set.append(neighbor)

                    global closes
                    closes.append(neighbor)

        return open_set, closed_set, current_node, final_path
