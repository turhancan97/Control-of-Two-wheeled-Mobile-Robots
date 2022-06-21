%% Kinematic Simulation of Three Land Based Mobile Robots
clc 
clear all
close all
% Add here your path (We need to take functions from previous folder)
addpath('C:\Users\Asus\Desktop\Master-Lectures\3rd Semester\Design-of-multi-agent-systems\Project\2nd_project\Control-of-Two-wheeled-Mobile-Robots');
%% Scripts
%% Inputs for model function
% theta_i = 0; % orientation
v_i = 1; % linear velocity
w_i = 1; % angular velocity
%% simulation parameter (To Create the Concept of Time)
total_time = 5;  % (seconds)
dt = 0.5; % sample sizes
time_stamp = total_time/dt; % total sample that we will see in simulation
i = 1; % start i from 1
%% Robot 1
x_pos_1 = zeros(3, time_stamp); % to store each positon of robot 1 by each sample
point_1 = [0;0;0]; % [x_1, y_1, theta_1]

%% Robot 2
x_pos_2 = zeros(3, time_stamp); % to store each positon of robot 2 by each sample
point_2 = [1;1;0]; % [x_2, y_2, theta_2]

%% Robot 3
x_pos_3 = zeros(3, time_stamp); % to store each positon of robot 3 by each sample
point_3 = [2;2;0]; % [x_3, y_3, theta_3]

%% Combine Robot Parameters
x_pos(:,:,1) = x_pos_1;
x_pos(:,:,2) = x_pos_2;
x_pos(:,:,3) = x_pos_3;

point(:,:,1) = point_1;
point(:,:,2) = point_2;
point(:,:,3) = point_3;
%% Motion
%% loop starts here - Robot move by time
while i <= time_stamp
    for j = 1:3 % loop for each robot (3 robots)
        x_pos(:,i,j) = point(:,:,j); % add the positions to the storage
        q_dot(:,:,j) = model(point(3,j),v_i,w_i); % kinematics calculation 
        point(1:2,j) = point(1:2,j) + q_dot(1:2,j) * dt; % move the robot's position
        point(3,j) = point(3,j) + q_dot(3,j) * dt; % move the robot's orientation
        if j == 1 % condition for each robot to plot their trajectory
            scatter(x_pos(1,i,j),x_pos(2,i,j),"b*") % blue - robot 1
        elseif j == 2
            scatter(x_pos(1,i,j),x_pos(2,i,j),"r*") % red - robot 2
        else 
            scatter(x_pos(1,i,j),x_pos(2,i,j),"g*") % green - robot 3
        end
        hold on % hold the previous graph
        pause(0.05) % delay
        %disp(point)
    end
    i = i + 1; % next iteration
end
grid on
xlabel('x,[m]'); ylabel('y,[m]');
title("Trajectory of the Three Robots",'FontSize',15)
legend('Mobile Robots1','Mobile Robots2','Mobile Robots3','Location',"southeast")