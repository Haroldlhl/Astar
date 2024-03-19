from PIL import Image
import numpy as np
# 输入输出以及关键参数
continuous_map_name = "picture/binary.png"


# 打开原始二值图像
img = Image.open(continuous_map_name)

# 生成图像矩阵, 取值0-255，灰度图像，大小取决于png图片分辨率
map_grid = np.array(img)
np.save('origin_map.npy', map_grid)

# 输入原始图像矩阵、期望大小，对其进行大小调整，输出二值矩阵
def map_resize(map_grid, width, height)
