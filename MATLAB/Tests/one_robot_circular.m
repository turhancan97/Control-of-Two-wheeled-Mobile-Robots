%% Kinematic Simulation of Single Land Based Mobile Robot
clc 
clear all
close all
% Add here your path (We need to take functions from previous folder)
addpath('C:\Users\Asus\Desktop\Master-Lectures\3rd Semester\Design-of-multi-agent-systems\Project\2nd_project\Control-of-Two-wheeled-Mobile-Robots');
%% Scripts
%% Inputs for model function
theta_i = 0; % orientation
v_i = 1; % linear velocity
w_i = 1; % angular velocity
%% simulation parameter (To Create the Concept of Time)
total_time = 5;  % (seconds)
dt = 0.1; % sample sizes
time_stamp = total_time/dt; % total sample that we will see in simulation
i = 1; % start i from 1
%% initial conditions
x_pos = zeros(3, time_stamp); % to store each positon by each sample
point = [0;0;0]; % [x, y, theta]
%% Motion
%% loop starts here - Robot move by time
while i <= time_stamp
    x_pos(:,i) = point; % add the positions to the storage
    q_dot = model(theta_i,v_i,w_i); % kinematics calculation 
    point(1:2) = point(1:2) + q_dot(1:2) * dt; % move the robot's position
    theta_i = theta_i + q_dot(3) * dt; % move the robot's orientation
    scatter(x_pos(1,i),x_pos(2,i),"b*") % plot the position
    hold on % hold the previous graph
    pause(0.1) % delay
    i = i + 1; % next iteration
end
grid on
xlabel('x,[m]'); ylabel('y,[m]');
title("Trajectory of the Robot",'FontSize',15)
legend('Mobile Robot','Location',"southeast")