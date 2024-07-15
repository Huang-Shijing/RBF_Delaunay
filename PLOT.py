import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

def call_back(event):
    axtemp = event.inaxes
    if axtemp is None:
        return

    x_min, x_max = axtemp.get_xlim()
    y_min, y_max = axtemp.get_ylim()
    xfanwei = (x_max - x_min) / 10
    yfanwei = (y_max - y_min) / 10

    if event.button == 'up':
        axtemp.set_xlim(x_min + xfanwei, x_max - xfanwei)
        axtemp.set_ylim(y_min + yfanwei, y_max - yfanwei)
    elif event.button == 'down':
        axtemp.set_xlim(x_min - xfanwei, x_max + xfanwei)
        axtemp.set_ylim(y_min - yfanwei, y_max + yfanwei)

    fig.canvas.draw_idle()

def plot_delaunay(delaunay_points , tri):
    global fig
    fig = plt.figure()
    fig.canvas.mpl_connect('scroll_event', call_back)
    
    # tri = Delaunay(delaunay_points)

    plt.triplot(delaunay_points[:,0], delaunay_points[:,1], tri.simplices)
    plt.plot(delaunay_points[:,0], delaunay_points[:,1],'o')
    plt.axis('equal')
    plt.axis([0, 1, -0.7, 0.7])
    plt.show()

def plot_grid(grid, x_coord, y_coord):
    global fig
    fig = plt.figure()
    fig.canvas.mpl_connect('scroll_event', call_back)
    plt.clf()  # 清除当前图形
    for i in range(len(grid)):
        node1 = int(grid[i, 0])
        node2 = int(grid[i, 1])
        xx = [x_coord[node1 - 1], x_coord[node2 - 1]]
        yy = [y_coord[node1 - 1], y_coord[node2 - 1]]
        #节点是否在物面上
        if grid[i, 6] == 3: 
            plt.plot(xx, yy, '-k', linewidth=1.5)
        else:
            plt.plot(xx, yy, '-r' )

    plt.axis('equal')
    plt.axis([ 0,  1 , -0.7 , 0.7])
    plt.show()
