import numpy as np
import pygicp
import gtsam

from gtsam.symbol_shorthand import X

def get_pose(pc_a, pc_b):
    aTb_estimate = gtsam.Pose3()
    aTb_matrix = pygicp.align_points(
            pc_a,
            pc_b,
            k_correspondences=15,
            num_threads=2
            )
    aTb = gtsam.Pose3(aTb_matrix)
    return aTb
