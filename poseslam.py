import gtsam
import rosbag 
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import pygicp

from gtsam.symbol_shorthand import X
from lidar_wrapper import LidarWrapper
from optimizer import Optimizer

graph = gtsam.NonlinearFactorGraph()
initial_estimate = gtsam.Values()

lidar = LidarWrapper(optimizer = Optimizer())
list_of_pointclouds = lidar.load_pointclouds('../rooster_2020-03-10-10-36-30_0.bag')

# graph = gtsam.NonlinearFactorGraph()
prior_model = gtsam.noiseModel.Diagonal.Sigmas((0.3, 0.3, 0.1))
graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), prior_model))

odometry_model = gtsam.noiseModel.Diagonal.Sigmas((0.2, 0.2, 0.1))
#
for i in range(len(list_of_pointclouds) - 1):
        lidar.lidar_callback()
        # a = i
        # b = i + 1
        # pc_a = np.array(list_of_pointclouds[a])
        # print(pc_a.shape)
        # pc_b = np.array(list_of_pointclouds[b])
        # factor, wTb_estimate = lidar.create_lidar_factor(a, b, pc_a, pc_b)
        # print("one done")
        # graph.add(factor)
#
# initial_estimate = gtsam.Values()
# for i in range(1, len(list_of_points) + 1):
#         initial_estimate.insert(i, gtsam.Pose3())
# print(initial_estimate)
