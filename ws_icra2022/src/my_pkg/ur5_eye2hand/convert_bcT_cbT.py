import numpy as np 
from scipy.spatial.transform import Rotation as Rot

bcT = np.matrix([[-7.34639719e-01, -6.27919076e-04,  6.78457138e-01, -1.08283672e+00],
 [-6.78432812e-01,  9.19848654e-03, -7.34604865e-01,  1.16241772e+00],
 [-5.77950645e-03, -9.99957496e-01, -7.18357448e-03,  4.39619904e-01],
 [ 0.00000000e+00,  0.00000000e+00 , 0.00000000e+00,  1.00000000e+00]])
R = bcT[:3,:3]
t = bcT[:3,3]
t2 = -R.T@t
print(t2)
cbT = np.zeros([4,4])
cbT[:3,:3] = R.T
for i in range(3):
    cbT[i,3] = t2[i,0]
print(cbT)
