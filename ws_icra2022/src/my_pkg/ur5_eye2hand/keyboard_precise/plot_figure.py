import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    log_x_array = np.load('log_x.npy')
    log_q = np.load('log_q.npy')
    log_qdot = np.load('log_qdot.npy')
    log_rdot = np.load('log_rdot.npy')
    log_dqdot = np.load('log_dqdot.npy')

    ros_freq = 30
    dx = np.array([[720],[540]])

    # task space velocity==============================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('task space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        if j<3:
            plt.ylabel('velocity (m/s)')
        else:
            plt.ylabel('angular velocity (rad/s)')

        plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]),np.reshape(np.array(log_rdot[j,:]),[-1,]) ,label = 'actual veloc')
        plt.legend()
    plt.savefig('log_r.jpg')

    # vision space position===============================================
    plt.figure()
    plt.plot(log_x_array[:,0], log_x_array[:,1],label = 'actual')
    plt.scatter(dx[0],dx[1],label = 'target',c='red')
    plt.legend()
    plt.title('vision space trajectory')
    plt.xlabel('x (pixel)')
    plt.ylabel('y (pixel)')
    plt.savefig('log_x.jpg')

    # vision space position verse time======================================
    fig = plt.figure(figsize=(20,8))
    plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]), log_x_array[:,0]-dx[0],label = 'x')
    plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]), log_x_array[:,1]-dx[1],label = 'y')
    plt.legend()
    # plt.title('vision space error')
    plt.xlabel('time (s)')
    plt.ylabel('error (pixel)')
    plt.savefig('log_x_t.jpg',bbox_inches='tight',dpi=fig.dpi,pad_inches=0.0)

    # joint space velocity=================================================
    plt.figure()
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')

        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),np.reshape(np.array(log_qdot[j,:]),[-1,]) ,label='actual joint velocity')
        plt.plot(np.linspace(0,np.shape(log_dqdot)[1]/ros_freq,np.shape(log_dqdot)[1]),log_dqdot[j,:].reshape(-1,), label = 'command joint velocity')
        plt.legend()
    plt.savefig('log_q.jpg')