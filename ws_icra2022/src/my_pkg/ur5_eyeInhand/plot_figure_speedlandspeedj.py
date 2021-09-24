import numpy as np
import matplotlib.pyplot as plt
import os


current_path = os.path.dirname(__file__)

# print(current_path)
# logq = np.ones([6,100],dtype=float)
# print(np.shape(logq)[0])
# ros_freq = 30
# plt.figure(figsize=(30,20))
# for j in range(np.shape(logq)[0]):
#     ax = plt.subplot(3, 2, j+1)
#     ax.set_title('joint %d velocity' % (j+1))
#     plt.xlabel('time[s]')
#     plt.ylabel('joint velocity[rad/s]')
#     plt.plot(np.linspace(0,np.shape(logq)[1]/ros_freq,np.shape(logq)[1]),logq[j,:])
# plt.savefig(os.path.join('/home/roboticslab/ur_ws/src/my_pkg/ur5/', 'logq.jpg'))

# dir = '/fig_speedl/'
dir = '/fig_null_space/'
# np.save(current_path+dir+'log_q.npy',logq)
# np.save('log_q.npy',logq)


log_rdot = np.load(current_path+dir+'log_rdot.npy')
log_drdot = np.load(current_path+dir+'log_drdot.npy')

ros_freq = 50 

# plot task space velocity
# plt.figure(figsize=(30,20))
# for j in range(6):
#     ax = plt.subplot(3, 2, j+1)
#     ax.set_title('task space velocity %d' % (j+1),fontsize=20)
#     plt.xlabel('time[s]')
#     plt.ylabel('task space velocity')

#     plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]),log_rdot[j,:].reshape(-1,))
#     plt.plot(np.linspace(0,np.shape(log_drdot)[1]/ros_freq,np.shape(log_drdot)[1]),log_drdot[j,:].reshape(-1,))
# # plt.savefig(os.path.join('/home/roboticslab/ur_ws/src/my_pkg/ur5/fig_speedl/', 'log_rdot.jpg'))
# plt.savefig(os.path.join('C:/Users/yan/Desktop/ur5_code/my_pkg/ur5/fig_null_space/', 'log_rdot.jpg'))

# plot task space position
dr = [0.476,-0.140, 0.451]
plt.figure(figsize=(30,20))
for j in range(6):
    ax = plt.subplot(3, 2, j+1)
    ax.set_title('task space velocity %d' % (j+1),fontsize=20)
    plt.xlabel('time[s]')
    plt.ylabel('task space position')

    plt.plot(np.linspace(0,np.shape(log_r)[1]/ros_freq,np.shape(log_r)[1]),log_r[j,:].reshape(-1,))
    plt.plot(np.linspace(0,np.shape(log_r)[1]/ros_freq,np.shape(log_r)[1]),dr[j,:]*np.ones(np.shape(log_r)[1]))
# plt.savefig(os.path.join('/home/roboticslab/ur_ws/src/my_pkg/ur5/fig_speedl/', 'log_r.jpg')) # on ubuntu
plt.savefig(os.path.join('C:/Users/yan/Desktop/ur5_code/my_pkg/ur5/fig_null_space/', 'log_r.jpg')) # on windows