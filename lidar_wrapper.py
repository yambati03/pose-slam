import gtsam
import rosbag 
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import pygicp

from gtsam.symbol_shorthand import X
from threading import Lock

# large chunks of the code below are taken from the following github repo
# https://github.gatech.edu/GeorgiaTechLIDARGroup/A1_SLAM/blob/510ecc2d55cfc3ec97e24a0752bd6f42ec3022e5/src/a1_slam/src/sensors/lidar3D.py
class LidarWrapper:
    def __init__(self, optimizer):
        self.state_index = 1
        self.optimizer = optimizer
        self.submap_clouds = []
        self.correspondence_threshold = 1.0
        self.icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.ones((6,)))
        self.baseTlaser = gtsam.Pose3()
        self.list_of_pointclouds = []
        self.lidar3d_lock = Lock()
    
    def load_pointclouds(self, path_to_bag: str):
        bag = rosbag.Bag(path_to_bag)
        topics=['/os1_cloud_node/points']

        count = 0
        for topic, msg, t in bag.read_messages(topics=topics):
                points = []
                for p in pc2.read_points(msg, skip_nans=True):
                        points.append((p[0], p[1], p[2]))
                if count % 10 == 0:
                        self.list_of_pointclouds.append(points)
                if count == 100:
                        break
                count += 1
                print(f'Percent progress: {count/100 * 100}%')
        return self.list_of_pointclouds
    def create_lidar_factor(self,
                            a: int,
                            b: int,
                            cloud_a: np.ndarray,
                            cloud_b: np.ndarray,
                            aTb_estimate=None):
        """Creates an odometry factor from LIDAR measurements to be
        added to the factor graph.
        Args:
            a: The previous state index of the odometry factor.
            b: The subsequent state index of the odometry factor.
            cloud_a: The LIDAR cloud associated with state a.
            cloud_b: The LIDAR cloud associated with state b.
            aTb_estimate: Initial guess for ICP.
        Returns:
            factor: An odometry factor to be added to the factor graph.
            wTb_estimate: Initial estimate if a new state, None otherwise.
        """

        # Calculate the ICP initial estimate if not given.
        if not aTb_estimate:
            if self.optimizer.results.exists(X(b)):
                wTa = self.optimizer.results.atPose3(X(a))
                wTb = self.optimizer.results.atPose3(X(b))
                aTb_estimate = wTa.between(wTb)
            elif a == 0 and b == 1:
                aTb_estimate = gtsam.Pose3()
            else:
                wTp = self.optimizer.results.atPose3(X(a-1))
                wTq = self.optimizer.results.atPose3(X(b-1))
                aTb_estimate = wTp.between(wTq)
        # Use multithreaded GICP to calculate the rigid body transform.
        aTb_matrix = pygicp.align_points(
            cloud_a,
            cloud_b,
            max_correspondence_distance=self.correspondence_threshold,
            initial_guess=aTb_estimate.matrix(),
            k_correspondences=15,
            num_threads=2
        )

        # Convert back into Pose3 object.
        aTb = gtsam.Pose3(aTb_matrix)
        factor = gtsam.BetweenFactorPose3(
            X(a), X(b), aTb, self.icp_noise_model)

        # Calculate the wTb pose estimate.
        wTa = self.optimizer.results.atPose3(X(a))
        wTb_estimate = wTa.compose(aTb)

        return factor, wTb_estimate
    
    def lidar_callback(self):
        """Processes the message from the LIDAR and create a IMU factor.
        Args:
            msg: a PointCloud2 ROS message.
            optimizer: an Optimizer object to add the factors and initial estimates.
            imu: if not None, an Imu object to use as an initial estimate for ICP.
        """

        # Calculate the aTb initial estimate from IMU if available.
        aTb_estimate = None
        index_a, index_b = self.state_index - 1, self.state_index


        cloud_b = self.list_of_pointclouds[index_b] 
        # cloud_b = self.baseTlaser.transformFrom(cloud_b)

        # Ignore if is the first received measurement.
        if len(self.submap_clouds) == 0:
            self.submap_clouds.append(cloud_b)
            return
        cloud_a = self.submap_clouds[-1]

        # Create the LIDAR factor, add to the optimizer, and optimize.
        factor, wTb_estimate = self.create_lidar_factor(
            index_a,
            index_b,
            cloud_a,
            cloud_b,
            aTb_estimate
        )
        self.optimizer.add_factor(factor, (X(index_b), wTb_estimate))
        self.optimizer.optimize()
        self.submap_clouds.append(cloud_b)

        # Create skip connections to create a denser graph.
        with self.lidar3d_lock:
            submap = self.submap_clouds.copy()
        if len(submap) > 2:
            self.optimizer.optimize()
        self.state_index += 1


