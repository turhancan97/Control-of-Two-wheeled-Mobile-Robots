import math
import numpy as np

R_ = 1.2
r_ = 0.2

# %% APF
def dV(inp):
    delta_ = 0.001
    R = R_
    r = r_
    l = inp
    if (l < r):
        out = 0
    elif (l > R-delta_):
        out = 0
    else:
        out = -1*(R-r)*math.exp((l-r)/(l-R))/(-1+math.exp((l-r)/(l-R)))**2/(l-R)**2;
    return out

def dVxy(inp):
    inp = np.array(inp)
    pxy = inp[0:2]
    oxy = inp[2:4]

    distvec = pxy-oxy
    l = np.linalg.norm(distvec)
    B = dV(l)

    if l > 0:
        dir = distvec/l
        grad = B * dir
    else:
        grad = np.array([0,0]).reshape(1,2) # [0 0]

    out = grad
    return out

## Function for error with Obstacle Avoidance
# Algorithm from Canuda et. al. (1994)

def error_obs(x_r,y_r,th_r, x, y, theta_i,dV_x,dV_y):
    M = np.array([np.cos(theta_i),np.sin(theta_i),0,
        -1*np.sin(theta_i),np.cos(theta_i),0,
        0, 0, 1]).reshape(3,3)
    c = np.array([x_r - x, y_r - y, th_r - theta_i]).reshape(3,1)
    c = c - np.array([dV_x,dV_y,0]).reshape(3,1)
    e = np.matmul(M,c)
    return e

## Model of the System
# The kinematic model of the two-wheeled mobile robot  
# is given by the function
def model(theta_i,v_i,w_i):
    T = np.array([np.cos(theta_i),0,np.sin(theta_i),0,0,1]).reshape(3,2)
    u = np.array([v_i, w_i]).reshape(2,1)
    q_dot = np.matmul(T,u)
    return q_dot

# %% Gain of the System
def k_gain(k1, k2, k3):
    return k1,k2,k3