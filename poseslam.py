import gtsam
import numpy as np
import pygicp

from gtsam.symbol_shorthand import X
from lidar_wrapper import LidarWrapper
from optimizer import Optimizer
from utils import get_pose

lidar = LidarWrapper(optimizer = Optimizer())
list_of_pointclouds = lidar.load_pointclouds('../rooster_2020-03-10-10-36-30_0.bag')

graph = gtsam.NonlinearFactorGraph()
prior_model = gtsam.noiseModel.Diagonal.Sigmas(np.random.rand(6, 1))

graph.add(gtsam.PriorFactorPose3(X(1), gtsam.Pose3(), prior_model))

odometry_model = gtsam.noiseModel.Diagonal.Sigmas((0.2, 0.2, 0.1))

for i in range(len(list_of_pointclouds) - 1):
        lidar.lidar_callback()
        a = i
        b = i + 1
        print(f"{a} {b} before function call")
        pc_a = np.array(list_of_pointclouds[a])
        pc_b = np.array(list_of_pointclouds[b])
        factor, wTb_estimate = lidar.create_lidar_factor(a, b, pc_a, pc_b)
        print("one done")
        graph.add(factor)

# for i in range(len(list_of_pointclouds) - 1):
#         a = i + 1
#         b = i + 2
#         pc_a = np.array(list_of_pointclouds[a - 1])
#         pc_b = np.array(list_of_pointclouds[b - 1])
#         aTb = get_pose(pc_a, pc_b)
#         print(f'{i*10}%')
#         factor = gtsam.BetweenFactorPose3(X(a), X(b), aTb, prior_model)
#         graph.add(factor)
#
# initial_estimate = gtsam.Values()
# for i in range(1, len(list_of_pointclouds) + 1):
#         initial_estimate.insert(i, gtsam.Pose3())
# print(initial_estimate)
# graph.saveGraph('graph.dot', initial_estimate)
# optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate)
# result = optimizer.optimize()
# print("Final Result:\n{}".format(result))
# marginals = gtsam.Marginals(graph, result)

