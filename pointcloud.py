import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from mpl_toolkits import mplot3d

bag = rosbag.Bag('../rooster_2020-03-10-10-36-30_0.bag')

topics=['/os1_cloud_node/points']


list_of_points = []
count = 0
for topic, msg, t in bag.read_messages(topics=topics):
    points = []
    for p in pc2.read_points(msg, skip_nans=True):
        points.append((p[0], p[1], p[2]))
    if count % 10 == 0:
        list_of_points.append(points)
    count += 1
    print(f'Percent progress: {count/1670 * 100}%')

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(*zip(*list_of_points[0]))
# ax.view_init(elev=75,azim=47)
# plt.show()

i = 0
for points in list_of_points:
    # plot each one on its own graph like the commented example above
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*zip(*points))
    ax.view_init(elev=75,azim=47)
    plt.savefig(f'fig/pointcloud_{i}.png')
    plt.figure().clear()
    plt.close()
    plt.cla()
    plt.clf()
    fig.clear()
    fig.clf()
    i += 1
    print(f'Percent progress: {i/len(list_of_points) * 100}%')

# i = 0
# for point_list in list_of_points:
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     x_vals = [p[0] for p in list_of_points[0]]
#     y_vals = [p[1] for p in list_of_points[0]]
#     z_vals = [p[2] for p in list_of_points[0]]
#     ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')
#     ax.view_init(elev=75, azim=47)
#     plt.savefig(f'fig/pointcloud{i}.png')
#     i += 1
#     plt.figure().clear()
#     plt.close()
#     plt.cla()
#     plt.clf()
#     print("Progress: {}/{}".format(i, len(list_of_points)))
