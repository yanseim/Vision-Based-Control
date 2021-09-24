import numpy as np
from scipy.spatial.transform import Rotation as R

if __name__=="__main__":
    quat = [ -0.0639317099704, -0.234710220328, -0.908551727516, 0.339643353183]
    r = R.from_quat(quat)
    print(r.as_matrix())