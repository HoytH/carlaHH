import pandas as pd
import matplotlib.pyplot as plt

# pc = pd.read_csv("/home/hoyt/Documents/Code/carlaHH/HHClient/HH/build/lidar_data.xyz", delimiter=' ')
# fig = plt.figure(figsize=(12, 12))
# # ax = fig.add_subplot(projection='3d')
# ax = fig.add_subplot()

# ax.scatter(pc['Y'], pc['Z'], c=pc['T'])
# ax.set_aspect('equal')
# plt.show()

import matplotlib.pyplot as plt
import networkx as nx

class Point:
    def __init__(self, x, y, category):
        self.x = x
        self.y = y
        self.category = category

def create_points():
    """
    Creates a list of Point objects with random coordinates and categories.
    """
    pc = pd.read_csv("/home/hoyt/Documents/Code/carlaHH/HHClient/HH/build/lidar_data.xyz", delimiter=' ')
    points = []
    for row in pc.iloc():
        points.append(Point(row.Y, row.Z, row['T']))
    return points


def plot_points(points):
    categories = set(point.category for point in points)
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    color_map = {}
    for idx, cat in enumerate(categories):
        color_map[cat] = colors[idx % len(colors)]

    for point in points:
        plt.scatter(point.x, point.y, color=color_map[point.category], label=point.category)
        print(point)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Points')
    plt.legend()
    plt.grid(True)
    plt.show()

def on_click(event):
    global selected_points, fig, ax
    x, y = event.xdata, event.ydata
    if event.button == 1:
        ax.plot(x, y, 'ro')
        selected_points.append((x, y))
        if len(selected_points) == 2:
            fig.canvas.mpl_disconnect(cid)
            plt.close(fig)

def get_shortest_path(points, start, end):
    G = nx.Graph()
    for point in points:
        G.add_node((point.x, point.y))

    for i in range(len(points)):
        for j in range(i+1, len(points)):
            if points[i].category == points[j].category:
                G.add_edge((points[i].x, points[i].y), (points[j].x, points[j].y),
                           weight=((points[i].x - points[j].x)**2 + (points[i].y - points[j].y)**2)**0.5)

    shortest_path = nx.shortest_path(G, source=(start.x, start.y), target=(end.x, end.y), weight='weight')
    return shortest_path

def main():
    points = create_points()
    plot_points(points)

    global selected_points, fig, ax, cid
    selected_points = []
    fig, ax = plt.subplots()
    cid = fig.canvas.mpl_connect('button_press_event', on_click)
    plt.show()

    start_point = None
    end_point = None
    for point in points:
        if (point.x, point.y) == selected_points[0]:
            start_point = point
        elif (point.x, point.y) == selected_points[1]:
            end_point = point

    shortest_path = get_shortest_path(points, start_point, end_point)

    print("Shortest Path:", shortest_path)

if __name__ == "__main__":
    main()
