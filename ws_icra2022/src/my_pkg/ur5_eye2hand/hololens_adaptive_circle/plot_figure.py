import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    log_x_array = np.load('log_x.npy')
    log_q = np.load('log_q.npy')
    log_qdot = np.load('log_qdot.npy')
    log_rdot = np.load('log_rdot.npy')
    log_dqdot = np.load('log_dqdot.npy')
    log_dx = np.load('log_dx.npy')
    log_theta_k = np.load('log_theta_k.npy')
    log_z_hat_array = np.load('log_z_hat.npy')
    

    ros_freq = 30
    dx = np.array([[720],[540]])

    # task space velocity==============================================
    plt.figure(figsize=(40,30))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('task space velocity %d' % (j+1),fontsize=10)
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
    plt.plot(log_dx[:,0],log_dx[:,1],":",label = 'desired')
    plt.legend()
    plt.title('vision space trajectory')
    plt.xlabel('x (pixel)')
    plt.ylabel('y (pixel)')
    plt.savefig('log_x.jpg')

    # joint space velocity=================================================
    plt.figure()
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')

        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),np.reshape(np.array(log_qdot[j,:]),[-1,]) )
        # plt.plot(np.linspace(0,np.shape(log_drdot)[1]/ros_freq,np.shape(log_drdot)[1]),log_drdot[j,:].reshape(-1,))
    plt.savefig('log_q.jpg')

    # theta_k========================================
    plt.figure()
    for j in range(15):
        lab = r'$\theta_k('+str(j+1)+')$'
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),    log_theta_k_array[:,j]-log_theta_k_array[0,j],label = lab)
        plt.xlabel('time (s)')
        plt.title(r'elements of $\hat \theta_k$')
        plt.legend()
    plt.savefig('log_thetak.jpg')

    # z_hat===================================
    plt.figure()
    plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]), np.reshape(log_z_hat_array,[-1,]))
    plt.title(r'$\hat z$')
    plt.xlabel('time (s)')
    plt.ylabel('depth (m)')
    plt.savefig('z_hat.jpg')

    