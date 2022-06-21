%% Error with Obstacle Avoidance
% algorithm from Canudas et al. (1994)
function e = error_obs(x_r,y_r,th_r, x, y, theta_i,dV_x,dV_y)
M = [cos(theta_i) sin(theta_i) 0;
    -sin(theta_i) cos(theta_i) 0;
    0 0 1];
c = [x_r - x; y_r - y; th_r - theta_i];
c = c - [dV_x;dV_y;0];
e = M * c;
end