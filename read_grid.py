import numpy as np

# 文件路径
file_path = 'naca0012-tri.cas'
# 读取文件内容
with open(file_path, 'r') as file:
    lines = file.readlines()

# 提取第22行到第719行的数据
target_lines_coord = lines[21:719]
# 初始化一个空的列表来存储坐标对
coordinates = []
# 遍历提取的行，分割字符串，并转换为浮点数
for line in target_lines_coord:
    parts = line.split()
    if len(parts) == 2:  # 确保每一行确实有两个部分
        x, y = float(parts[0]), float(parts[1])
        coordinates.append([x, y])
# 将列表转换为 NumPy 矩阵
Coord = np.array(coordinates)


# 提取第728行至第2633行的数据（注意：行索引是从0开始的）
# 所以我们需要的是第727行到第2632行的数据
target_lines_grid1 = lines[727:2633]
# 初始化一个空列表来存储转换后的10进制数据
grid1 = []
# 遍历目标行，提取每行的数据并转换
for line in target_lines_grid1:
    parts = line.split()
    if parts:  # 确保行不为空
        # 提取这一行的所有部分，并将它们从16进制转换为10进制
        decimal_row = [int(part, 16) for part in parts]
        grid1.append(decimal_row)
# 将列表转换为 NumPy 矩阵
grid1 = np.array(grid1)
# 使用前两列数据生成新的矩阵
new_grid1 = grid1[:, :2]
# 初始化一个列表来存储第三列的值，每个值都是2，内场点
third_column = [2] * len(new_grid1)
# 将第三列添加到新的矩阵中
new_grid1 = np.column_stack((new_grid1 , third_column))

# 提取第2639行到第2674行的数据（注意：行索引是从0开始的）
# 所以我们需要的是第2638行到第2673行的数据
target_lines_grid2 = lines[2638:2674]
# 初始化一个空列表来存储转换后的10进制数据
grid2 = []
# 遍历目标行，提取每行的数据并转换
for line in target_lines_grid2:
    parts = line.split()
    if parts:  # 确保行不为空
        # 提取这一行的所有部分，并将它们从16进制转换为10进制
        decimal_row = [int(part, 16) for part in parts]
        grid2.append(decimal_row)
# 将列表转换为 NumPy 矩阵
grid2 = np.array(grid2)
# 使用前两列数据生成新的矩阵
new_grid2 = grid2[:, :2]
# 初始化一个列表来存储第三列的值，每个值都是9,远场边界点
third_column = [9] * len(new_grid2)
# 将第三列添加到新的矩阵中
new_grid2 = np.column_stack((new_grid2 , third_column))

# 提取第2680行到第2737行的数据（注意：行索引是从0开始的）
# 所以我们需要的是第2679行到第2736行的数据
target_lines_grid3 = lines[2679:2737]
# 初始化一个空列表来存储转换后的10进制数据
grid3 = []
# 遍历目标行，提取每行的数据并转换
for line in target_lines_grid3:
    parts = line.split()
    if parts:  # 确保行不为空
        # 提取这一行的所有部分，并将它们从16进制转换为10进制
        decimal_row = [int(part, 16) for part in parts]
        grid3.append(decimal_row)
# 将列表转换为 NumPy 矩阵
grid3 = np.array(grid3)
# 使用前两列数据生成新的矩阵
new_grid3 = grid3[:, :2]
# 初始化一个列表来存储第三列的值，每个值都是3,物面边界点
third_column = [3] * len(new_grid3)
# 将第三列添加到新的矩阵中
new_grid3 = np.column_stack((new_grid3 , third_column))
wallNodes = new_grid3[:,0]

Grid = np.concatenate((new_grid1,new_grid2,new_grid3))
# print(Grid)