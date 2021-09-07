from scipy.spatial.transform import Rotation as R



r = R.from_quat([-0.256585935223, 0.661630897921, -0.655411637503, 0.258541675919])
theta_z = [0]*13
theta_z[-4:] = [6.78457138e-01,-7.34604865e-01,-7.18357448e-03,1.2]



print(theta_z)
print(r.as_matrix())