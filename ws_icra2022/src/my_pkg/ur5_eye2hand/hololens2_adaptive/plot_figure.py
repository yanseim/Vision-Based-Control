import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    log_x_array = np.load('log_x.npy')
    log_q = np.load('log_q.npy')
    log_qdot = np.load('log_qdot.npy')
    log_rdot = np.load('log_rdot.npy')
    log_dqdot = np.load('log_dqdot.npy')
    # log_dx = np.load('log_dx.npy')
    log_theta_k_array = np.load('log_theta_k.npy')
    log_theta_z_array = np.load('log_theta_z.npy')
    log_z_hat_array = np.load('log_z_hat.npy')
    log_d_array = np.load('log_d.npy')

    ros_freq = 30
    dx = np.array([[720],[540]])

    # task space velocity==============================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('Cartesian space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        if j<3:
            plt.ylabel('velocity (m/s)', fontsize = 18)
        else:
            plt.ylabel('angular velocity (rad/s)',fontsize = 18)

        plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]),np.reshape(np.array(log_rdot[j,:]),[-1,]) )
        # plt.legend()
    plt.savefig('log_r.jpg')

    # vision space position===============================================
    fig = plt.figure()
    plt.scatter(dx[0],dx[1], c='red',label = 'target')
    plt.plot(log_x_array[:,0], log_x_array[:,1],label = 'actual')
    plt.legend()
    # plt.title('vision space trajectory')
    plt.xlabel('x (pixel)')
    plt.ylabel('y (pixel)')
    plt.yticks()
    plt.savefig('log_x.jpg',bbox_inches='tight',dpi=fig.dpi,pad_inches=0.0)

    # vision space position verse time======================================
    fig = plt.figure(figsize=(20,8))
    plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]), log_x_array[:,0]-dx[0],label = 'x')
    plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]), log_x_array[:,1]-dx[1],label = 'y')
    plt.legend()
    # plt.title('vision space error')
    plt.xlabel('time (s)')
    plt.ylabel('error (pixel)')
    plt.savefig('log_x_t.jpg',bbox_inches='tight',dpi=fig.dpi,pad_inches=0.0)

    # intention=========================================================
    plt.figure()
    for j in range(log_d_array.shape[1]):
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),log_d_array[:,j],label = 'intention'+str(j+1))
    plt.legend()
    plt.xlabel('time (s)')
    plt.ylabel(' intention(rad/s)')
    plt.savefig('log_d.jpg')

    # joint ============================================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),np.reshape(np.array(log_q[j,:]),[-1,]) )
        if j==4:
            # plt.ylim([1.2,1.5])
	    pass
        elif j==5:
            plt.ylim([2.4,2.7])
    plt.savefig('log_q.jpg')

    # joint space velocity=================================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')

        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),np.reshape(np.array(log_qdot[j,:]),[-1,]) )
        plt.plot(np.linspace(0,np.shape(log_dqdot)[1]/ros_freq,np.shape(log_dqdot)[1]),log_dqdot[j,:].reshape(-1,))
    plt.savefig('log_qdot.jpg')

    # theta_k========================================
    plt.figure()
    for j in range(15):
        lab = r'$\theta_k('+str(j+1)+')$'
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),    log_theta_k_array[:,j]-log_theta_k_array[0,j],label = lab)
        plt.xlabel('time (s)')
        plt.title(r'elements of $\hat \theta_k$')
        plt.legend()
    plt.savefig('log_thetak.jpg')

    # theta_z========================================
    plt.figure()
    for j in range(13):
        lab = r'$\theta_z('+str(j+1)+')$'
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),    log_theta_z_array[:,j]-log_theta_z_array[0,j],label = lab)
        plt.xlabel('time (s)')
        plt.title(r'elements of $\hat \theta_z$')
        plt.legend()
    plt.savefig('log_thetaz.jpg')

    # z_hat===========================================
    plt.figure()
    plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]), np.reshape(log_z_hat_array,[-1,]),label = 'z_hat')
    plt.legend()
    plt.title('z_hat')
    plt.xlabel('time (s)')
    plt.ylabel('depth (m)')
    plt.savefig('z_hat.jpg')

    
