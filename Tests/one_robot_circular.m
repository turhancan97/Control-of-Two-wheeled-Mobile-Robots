clc 
clear all
addpath('C:\Users\Asus\Desktop\Master-Lectures\3rd Semester\Design-of-multi-agent-systems\Project\2nd_project\Control-of-Two-wheeled-Mobile-Robots');
%% Scripts
theta_i = 0;
v_i = 1;
w_i = 1;

total_time = 5  % (seconds)
dt = 0.1 % sample sizes
time_stamp = total_time/dt
i = 1

x_pos = zeros(3, time_stamp)

point = [0;0;0]

while i <= time_stamp
    x_pos(:,i) = point;
    q_dot = model(theta_i,v_i,w_i);
    point(1:2) = point(1:2) + q_dot(1:2) * dt;
    theta_i = theta_i + q_dot(3) * dt;
    scatter(x_pos(1,i),x_pos(2,i),"b*")
    hold on
    pause(0.1)
    i = i + 1;
end