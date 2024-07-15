import numpy as np
from scipy.spatial import Delaunay
from locate import point_grid
from PLOT import plot_delaunay , plot_grid
from read_grid import Coord , wallNodes , Grid

xCoord = np.array([Coord[:, 0]]).T
yCoord = np.array([Coord[:, 1]]).T
nNodes = Coord.shape[0]
nWallNodes = len(wallNodes)

#参数
A = 0.6 #控制点椭圆长半径x轴方向
B = 0.3 #控制点椭圆短半径y轴方向

#delaunay背景网格
delaunay_index = []
for i in wallNodes:
    delaunay_index.append(int(i - 1))

#添加远场边界点
delaunay_index_far = delaunay_index.copy()
delaunay_index_far.extend([0, 9, 18, 27])
delaunay_index_far = sorted(delaunay_index_far)
delaunay_points = Coord[delaunay_index_far]

#添加控制点
control_points = []
for i in range(60):
    control_points.append([A * np.cos(i * 2 * np.pi / 60), B * np.sin(i * 2 * np.pi / 60)])
combined_points = np.vstack((delaunay_points, control_points))

tri = Delaunay(combined_points)

# plot_delaunay(combined_points , tri)

#关联计算网格节点与背景网格单元
pg = point_grid(Coord , tri)

e_points = [] #计算网格点在网格中的的定位系数
for i in range(nNodes):
    s = [] #三角形面积矩阵
    e = []
    for j in range(3):
        s.append(np.append(tri.points[tri.simplices[pg[i][1]][j]] , 1))
    ss = np.linalg.det(s) #三角形面积
    for j in range(3):
        se = s.copy() 
        se[j] = np.append(Coord[i] , 1)
        e.append(np.linalg.det(se) / ss) #计算定位系数
    e_points.append(e)

#物面运动
delaunay_points_new = delaunay_points.copy()
dy = 0.2 * np.sin(-2*np.pi * delaunay_points_new[:,0] )
dy = np.array([dy]).T
for i in range(4,len(delaunay_index_far)):
    delaunay_points_new[i,1] += dy[i][0]
    
combined_points_new = np.vstack((delaunay_points_new, control_points))

# plot_delaunay(combined_points_new , tri)

# 计算权重系数W
Fai = np.zeros((nWallNodes, nWallNodes))
P = np.zeros((nWallNodes, 3))
for i in range(nWallNodes):
    wall_index = delaunay_index[i]
    x1 = xCoord[wall_index]
    y1 = yCoord[wall_index]
    for j in range(nWallNodes):
        wall_index2 = delaunay_index[j]
        x2 = xCoord[wall_index2]
        y2 = yCoord[wall_index2]
        #距离加上1e-40防止除以零
        dis = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) + 1e-40
        Fai[i, j] = dis[0]**2 * np.log10(dis[0])
    P[i:] = [ 1 , x1[0] , y1[0]]
matrix_down = np.concatenate(( P , Fai ) , axis=1 )
matrix_up = np.concatenate(( np.zeros(( 3 , 3 )) , P.T ) , axis=1 )
matrix_combined = np.concatenate(( matrix_up , matrix_down ))
dy = dy[4:]
dy = np.concatenate(( np.zeros(( 3 , 1 )) , dy ))
W = np.dot(np.linalg.pinv(matrix_combined),dy)

# 利用W计算远场边界点与控制点的位移
fai = np.zeros((1, nWallNodes))
p = np.zeros((1, 3))
for i in range(len(combined_points_new)):
    xNode = combined_points_new[i][0]
    yNode = combined_points_new[i][1]
    if i in range(4 , len(delaunay_index_far)):
        continue
    for j in range(nWallNodes):
        wall_index = delaunay_index[j]
        xw = xCoord[wall_index]
        yw = yCoord[wall_index]
        dis = np.sqrt((xNode - xw)**2 + (yNode - yw)**2) + 1e-40
        fai[0, j] = dis[0]**2 * np.log10(dis[0])
    p[0, :] = [1, xNode, yNode]
    m = np.concatenate((p, fai), axis=1)
    dy = np.dot(m[0, :] , W)  
    combined_points_new[i][1] = combined_points_new[i][1] + dy[0]  

# plot_delaunay(combined_points_new , tri)

#通过变化后的delaunay网格得到计算网格节点的新坐标
compute_points = Coord.copy()
for i in range(nNodes):
    ymove = 0
    dis_ob = []
    for j in range(3):
        index = tri.simplices[pg[i][1]][j]
        ymove += (combined_points_new[index][1] - combined_points[index][1] ) * e_points[i][j]
    #添加衰减函数，通过点与物面及远场边界距离的比值来计算
    dis_far = min(10 - abs(compute_points[i][0]) , 10 - abs(compute_points[i][1]))
    for j in delaunay_index:
        dis_ob.append(np.sqrt((xCoord[j] - compute_points[i][0])**2 + (yCoord[j] - compute_points[i][1])**2))
    dis_ob = min(dis_ob)
    Ra = dis_far / (dis_far + dis_ob)
    compute_points[i][1] += ymove * Ra[0]
  
plot_grid(Grid , compute_points[:,0] , compute_points[:,1])