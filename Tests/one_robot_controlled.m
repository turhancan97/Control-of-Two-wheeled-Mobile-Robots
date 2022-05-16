clc 
clear all
addpath('C:\Users\Asus\Desktop\Master-Lectures\3rd Semester\Design-of-multi-agent-systems\Project\2nd_project\Control-of-Two-wheeled-Mobile-Robots');
%% Scripts
theta_i = 0;
v_i = 1;
w_i = 1;
v_r = 1;
w_r = pi;

total_time = 5;  % (seconds)
dt = 0.1; % sample sizes
time_stamp = total_time/dt;
i = 1;

x_pos = zeros(3, time_stamp);

point = [0;0;0];

position_r = [5;5;pi];
figure(1)
while i <= time_stamp
    x_pos(:,i) = point;
    q_dot = model(theta_i,v_i,w_i);
    point(1:2) = point(1:2) + q_dot(1:2) * dt;
    theta_i = theta_i + q_dot(3) * dt;
    point(3) = theta_i;
    scatter(x_pos(1,i),x_pos(2,i),"b*")
    hold on
    pause(0.1)
    e = error_function(position_r(1),position_r(2),position_r(3),point(1),point(2),theta_i)
    v_i = v_r * cos(e(3)) + 2 * e(1);
    w_i = w_r + (-2 * e(2) + (2*e(3)));
    er(:,i) = e;
    time(i) = i/10;
    i = i + 1;
end
subplot(2,2,1);
plot(er(1,:),"r")
title("Error in X Position")
subplot(2,2,2);
plot(er(2,:),"b")
title("Error in Y Position")
subplot(2,2,3);
plot(x_pos(1,:),x_pos(2,:),"black")
title("Actual Trajectory")
subplot(2,2,4);
plot(time,x_pos(3,:),"green")
title("Theta by Time")


