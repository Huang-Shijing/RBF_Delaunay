import numpy as np

#为了得到每个点的位置，需要遍历每个三角形，判断点是否在三角形内
pointlocation = []
def point_grid(Coord , tri):
    #这里是对每一个点进行定位
    for i in range(len(Coord)):
        flag = 1
        j = 0
        jlist = []
        #这里利用flag标记时候结束对三角形的遍历，j标记当前遍历的三角形
        while flag !=0 :
            jlist.append(j)
            vector01 = tri.points[tri.simplices[j][0]] - tri.points[tri.simplices[j][1]]
            vector02 = tri.points[tri.simplices[j][0]] - tri.points[tri.simplices[j][2]]
            vector0x = tri.points[tri.simplices[j][0]] - Coord[i]
            cross0102 = np.cross(vector01 , vector02)
            cross010x = np.cross(vector01 , vector0x)
            if np.dot(cross0102 , cross010x) > 0:
                cross020x = np.cross(vector02 , vector0x)
                cross0201 = - cross0102
                if np.dot(cross0201 , cross020x) > 0:
                    vector12 = tri.points[tri.simplices[j][1]] - tri.points[tri.simplices[j][2]]
                    vector10 = - vector01
                    vector1x = tri.points[tri.simplices[j][1]] - Coord[i]
                    cross1210 = np.cross(vector12 , vector10)
                    cross121x = np.cross(vector12 , vector1x)
                    if np.dot(cross1210 , cross121x) < 0:
                        #如果不在该三角形内，则根据三角形的邻接关系，遍历相邻的三角形
                        j = int(tri.neighbors[j][0])
                        #这里不确定是否在数学上不会发生该种情况
                        if j in jlist:
                            print("出错，循环遍历")
                        elif j == -1:
                            print("点在多边形外")
                    else:
                        flag = 0
                        pointlocation.append([i , j])
                elif np.dot(cross0201 , cross020x) == 0:
                    flag = 0
                    pointlocation.append([i , j])
                else:
                    j = int(tri.neighbors[j][1])
                    if j in jlist:
                        print("出错，循环遍历")
                    elif j == -1:
                        print("点在多边形外")
            elif np.dot(cross0102 , cross010x) == 0:
                flag = 0
                pointlocation.append([i , j])
            else:
                j = int(tri.neighbors[j][2])
                if j in jlist:
                    print("出错，循环遍历")
                elif j == -1:
                    print("点在多边形外")

    pointlocation.sort()

    return pointlocation
