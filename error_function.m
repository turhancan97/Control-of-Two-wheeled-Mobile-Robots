%% Error of the System

function e = error_function(x_r,y_r,th_r, x, y, theta_i)
M = [cos(theta_i) sin(theta_i) 0;
    -sin(theta_i) cos(theta_i) 0;
    0 0 1];
c = [x_r - x; y_r - y; th_r - theta_i];
e = M * c;
end