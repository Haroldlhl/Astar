from enum import Enum
import map_decrete  # 图像处理成灰度矩阵


# 定义重要参数
start_loc = (24, 33)
end_loc = (460, 127)
map_size = (500, 800)


class GridType(Enum):
    start = 0
    end = 1
    free = 2
    barrier = 3


map_colors = np.zeros(np.shape(map_grid), dtype=np.int8)    # 实际颜色地图，颜色取决于映射

# 定义地图的宽度和高度
MAP_HEIGHT, MAP_WIDTH = np.shape(map_grid)
cnt = 0
closes = []         # 闭集中的点



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
            node.r = rands[x][y]
            if node.sort == 'obstacle':
                img.putpixel((y, x), (0, 0, 0))
    img.save('binary11.png')

    global cnt
    path_astar = []
    while open_set:
        open_set, closed_set, current_node, final_path = \
            AStar.update_sets_once(open_set, closed_set, end_dot, nodes)

        # 地图更新

        # 结束时的输出处理
        if final_path:
            print("cnt=", cnt)
            print("The way found!!!")
            for dot in final_path:
                path_astar.append([dot.x, dot.y])
            break
        if len(closed_set) > MAP_WIDTH*MAP_HEIGHT/2:
            raise ValueError("close_set is too big, something wrong happened!!")
        cnt += 1

    for node in closes:
        img.putpixel((node.y, node.x), (255, 130, 71))
    for node in final_path:
        img.putpixel((node.y, node.x), (255, 0, 0))
    # break_dots = np.load('pic2_detect1.npy')
    # # for jj in range(len(break_dots)):
    # #     img.putpixel((break_dots[jj][1], break_dots[jj][0]), (0, 255, 0))
    # for jj in [0, 1, 2, -1]:
    #     img.putpixel((break_dots[jj][1], break_dots[jj][0]), (0, 255, 0))
    #
    # img.putpixel((START_LOC[1], START_LOC[0]), (0, 255, 0))
    # img.putpixel((END_LOC[1], END_LOC[0]), (0, 255, 0))

    img.save("astar_find1.png")
    img.show()
    path_astar = path_astar[::-1]
    return path_astar


path = np.array(run_astar())
np.save('path.npy', path)
print(path)



from PIL import Image


def image_trim():
    # 打开原始图片
    original_image = Image.open("map_continuous.png")  # 3482*2176

    fixed_image = Image.new("RGB", (800, 500))

    for width in range(800):
        for height in range(500):
            original_color = original_image.getpixel((width * 3482 // 800, height * 2176 // 500))
            fixed_image.putpixel((width, height), original_color)

    # 保存新图片
    fixed_image.save("fixed_image.bmp")
    fixed_image.show()


def get_binary():
    # yellow = (255, 230, 153)
    # green = (183, 222, 195)
    # blue = (156, 192, 250)
    # original_image = Image.open("fixed_image1.bmp")  # 800*500
    # gray_image = original_image.convert("L")
    # gray_image.show()
    # gray_image.save("gray_image")
    #
    # threshold = 130
    # # 使用point方法将灰度图片转换为二值图片
    # binary_image = gray_image.point(lambda x: 0 if x < threshold else 255, mode='1')
    # # 保存二值图片
    # binary_image.save("binary_image.bmp")
    # # 关闭原始灰度图片
    # gray_image.show()
    binary_image = Image.open("binary_image.bmp")
    binary3_image = Image.new("RGB", (800, 500))
    for width in range(800):
        for height in range(500):
            pixel_value = binary_image.getpixel((width, height))

            # 根据像素值设置RGB颜色
            if pixel_value == 0:
                rgb_color = (0, 0, 0)  # 黑色
            else:
                rgb_color = (255, 255, 255)  # 白色

            # 在新图像上设置相应的颜色
            binary3_image.putpixel((width, height), rgb_color)
    binary3_image.show()
    binary3_image.save("RGB_binary.bmp")


def get_pixel_local():
    binary_image = Image.open("RGB_binary.bmp")

    for width in range(520, 800):
        for height in range(500):
            original_color = binary_image.getpixel((width, height))
            if original_color[0] < 20 and original_color[1] < 20 and original_color[2] > 240:
                print((width, height))
                binary_image.putpixel((width, height), (0, 255, 0, 255))
                binary_image.show()
                return None


# (width, height)
start = (33, 24)
end1, end2, end3, end4, end5, end6 = (127, 460), (303, 258), (360, 434), (370, 32), (497, 198), (681, 420)

