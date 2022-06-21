import numpy as np
from functions import dV, dVxy, error_obs, model, k_gain, R_ , r_
import matplotlib.pyplot as plt
# %% simulation parameter
total_time = 70 # (seconds)
dt = 0.1; # step sizes
time_stamp = total_time/dt # simulation time
i = 0 # to start while loop
time = np.zeros([1,int(time_stamp)])

# %% mobile robot parameter and initial contitions
v_i = [0, 0, 0] # initial linear velocity of a followers (ignore first index)
w_i = [0, 0, 0] # initial angular velocity of a followers (ignore first index)

# %% Virtual Leader Parameters
x_pos_v = np.zeros([3, int(time_stamp)]) # store the position
point_v = [5,5,0] # starting point and orientation

# %% Robot 1 (follower) parameters
x_pos_1 = np.zeros([3, int(time_stamp)]) # store the position
point_1 = [8,8,(3*np.pi/2)];  # starting point and orientation

# %% Robot 2 (follower) parameters 
x_pos_2 = np.zeros([3, int(time_stamp)]) # store the position
point_2 = [10,10,(3*np.pi/2)] # starting point and orientation

# %% To store the error values
er = np.zeros([3, 3]);
er_f1 = np.zeros([3,int(time_stamp)])
er_f2 = np.zeros([3,int(time_stamp)])
# %% Target positions for the followers
target_r_1 = + 1
target_r_2 = target_r_1 + 1

# %% k parameters for control
k1,k2,k3 = k_gain(0.3,0.3,0.8)
# %% Trajectory Parameter
trajectory = "stadium" # % "stadium" % "circle"
# %% loop starts here
while i <= time_stamp-1:
    if trajectory == "stadium":
        if i % 120 < 40: # % first line
            v_r = 1 # % virtual linear velocity
            w_r = 0 # % virtual angular velocity
        elif i % 120 < 60: # % first arc 
            v_r = 1 # % virtual linear velocity
            w_r = np.pi/2 # % virtual angular velocity
        elif i % 120 < 100: # % second line
            v_r = 1 # % virtual linear velocity
            w_r = 0 # % virtual angular velocity
        elif i % 120 <= 119: # % second arc
            v_r = 1 # % virtual linear velocity
            w_r = np.pi/2 # % virtual angular velocity
    elif trajectory == "circle":
        v_r = 1 # % virtual linear velocity
        w_r = np.pi/6 # % virtual angular velocity
    # % Virtual Leader Process (1.robot işlemleri yapılıyor)
    x_pos_v[:,i] = point_v
    q_dot = model(point_v[2],v_r,w_r) # % time derivative of position
    point_v[0:2] = np.add(point_v[0:2],([q_dot[0][0]*dt,q_dot[1][0]*dt])) # % state (position) update
    point_v[2] = np.add(point_v[2],(q_dot[2][0] * dt)) # % state (orientation) update
    # % 1st and 2nd follower process (2. ve 3. robot işlemleri)
    for j in range(1,3,1): # % simulate the followers here 
        if j == 1: # % for 1st follower
            x_pos_1[:,i] = point_1
            q_dot = model(point_1[2],v_i[j],w_i[j]) # % time derivative of position
            point_1[0:2] = np.add(point_1[0:2],([q_dot[0][0]*dt,q_dot[1][0]*dt])) # % state (position) update
            point_1[2] = np.add(point_1[2],(q_dot[2][0] * dt)) # % state (orientation) update
            # % Obstacle Avoidance
            # % x_pos(coordinate, time step , robot number)
            xr = x_pos_1[:,i][0] # % x coordinate of 1st follower 
            yr = x_pos_1[:,i][1] # % y coordinate of 1st follower
            xobs1 = x_pos_v[:,i][0] # % x coordinate of virtual leader (Obstacle for followers)
            yobs1 = x_pos_v[:,i][1] # % y coordinate of virtual leader (Obstacle for followers)
            dV1 = dVxy([xr,yr,xobs1,yobs1]) # % APF function for virtual leader
            dV2 = dVxy([xr,yr,10.5,6.7]) # % APF function for static obstacle (x = 10.5, y = 6.7)
            dV3 = dVxy([xr,yr,9,7.15]) # % APF function for static obstacle (x = 9, y = 7.15)
            dV4 = dV1 + dV2 + dV3 # % sum of all obstacle
            # % Control Algorithm
            # % error_obs(x of virtual learder, y of virtual leader - 1, virtual leader theta, x of follower 1, y of follower 1, theta of follower 1, APF x, APF y)
            e = error_obs(x_pos_v[:,i][0],x_pos_v[:,i][1]-target_r_1,x_pos_v[:,i][2],x_pos_1[:,i][0],x_pos_1[:,i][1],x_pos_1[:,i][2],dV4[0],dV4[1])
            er[:,j] = e.reshape(3,) # % for control algorithm
            er_f1[:,i] = e.reshape(3,)
            # err(:,i,1) = e # % for plotting 
            v_i[j] = v_r * np.cos(er[2,j]) + k1 * (er[0,j]) # % Control Algorithm from Canudas et al. (1994)
            w_i[j] = w_r + (k2 * (er[1,j]) + (k3 *er[2,j])) # % Control Algorithm from Canudas et al. (1994)
        else:  # % for second follower
            x_pos_2[:,i] = point_2
            q_dot = model(point_2[2],v_i[j],w_i[j]) # % time derivative of position
            point_2[0:2] = np.add(point_2[0:2],([q_dot[0][0]*dt,q_dot[1][0]*dt])) # % state (position) update
            point_2[2] = np.add(point_2[2],(q_dot[2][0] * dt)) # % state (orientation) update
            # % Obstacle Avoidance
            # % x_pos(coordinate, time step , robot number)
            xr = x_pos_2[:,i][0] # % x coordinate of 2nd follower 
            yr = x_pos_2[:,i][1] # % y coordinate of 2nd follower
            xobs1 = x_pos_v[:,i][0] # % x coordinate of virtual leader (Obstacle for followers)
            yobs1 = x_pos_v[:,i][1] # % y coordinate of virtual leader (Obstacle for followers)
            xobs2 = x_pos_1[:,i][0] # % x coordinate of 1st follower (Obstacle for followers)
            yobs2 = x_pos_1[:,i][1] # % y coordinate of 1st follower (Obstacle for followers)
            dV1 = dVxy([xr,yr,xobs1,yobs1]) # % APF function for virtual leader
            dV2 = dVxy([xr,yr,xobs2,yobs2]) # % APF function for 1st follower
            dV3 = dVxy([xr,yr,10.5,6.7]) # % APF function for static obstacle (x = 10.5, y = 6.7)
            dV4 = dVxy([xr,yr,9,7.15]) # % APF function for static obstacle (x = 9, y = 7.15)
            dV5 = dV1 + dV2 + dV3 + dV4 # % sum of all obstacle
            # % Control Algorithm
            # % error_obs(x of virtual learder, y of virtual leader - 2, virtual leader theta, x of follower 2, y of follower 2, theta of follower 2, APF x, APF y)
            e = error_obs(x_pos_v[:,i][0],x_pos_v[:,i][1]-target_r_2,x_pos_v[:,i][2],x_pos_2[:,i][0],x_pos_2[:,i][1],x_pos_2[:,i][2],dV5[0],dV5[1])
            er[:,j] = e.reshape(3,) # % for control algorithm
            er_f2[:,i] = e.reshape(3,)
            # err(:,i,1) = e # % for plotting 
            v_i[j] = v_r * np.cos(er[2,j]) + k1 * (er[0,j]) # % Control Algorithm from Canudas et al. (1994)
            w_i[j] = w_r + (k2 * (er[1,j]) + (k3 *er[2,j])) # % Control Algorithm from Canudas et al. (1994)
    time[:,i] = i/10
    i = i + 1

# %% Plot Trajectories and Errors

fig, ax = plt.subplots(figsize=(18, 9), layout='constrained')
ax.plot(x_pos_v[0,:],x_pos_v[1,:],"b",label='Virtual Leader')
ax.plot(x_pos_1[0,:],x_pos_1[1,:],"r",label='First Follower')
ax.plot(x_pos_2[0,:],x_pos_2[1,:],"g",label='Second Follower')
ax.scatter(10.5,6.35,60,"black","x",label='Static Obstacle 1')
ax.scatter(9,7.15,60,"black","x",label='Static Obstacle 2')
ax.set_title('Trajectory Simulation',fontsize=15)
ax.set_xlabel('x [m]',fontsize=20)
ax.set_ylabel('y [m]',fontsize=20)
ax.grid(linestyle='--', linewidth=1) 
ax.legend()

fig, axs = plt.subplots(1, 2,figsize=(9, 4.5), layout='constrained')
axs[0].plot(time.reshape(time.shape[1],),er_f1[0,:],'r',label="x")
axs[0].plot(time.reshape(time.shape[1],),er_f1[1,:],'b',label="y")
axs[0].plot(time.reshape(time.shape[1],),er_f1[2,:],'g',label="theta")
axs[0].set_title('Error of First Follower',fontsize=15)
axs[1].plot(time.reshape(time.shape[1],),er_f2[0,:],'r',label="x")
axs[1].plot(time.reshape(time.shape[1],),er_f2[1,:],'b',label="y")
axs[1].plot(time.reshape(time.shape[1],),er_f2[2,:],'g',label="theta")
axs[1].set_title('Error of Second Follower',fontsize=15)

for ax in axs.flat:
    ax.set_xlabel('Time [s]',fontsize=15)
    ax.set_ylabel('Error [x,y,theta]',fontsize=15) 

# Hide x labels and tick labels for top plots and y ticks for right plots.
for ax in axs.flat:
    ax.label_outer()
    
axs[0].legend()
axs[1].legend()
plt.show()