%% Kinematic Simulation of a Single Land Based Mobile Robot with Control
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
%% virtual leader parameters
v_r = 1; % reference linear velocity
w_r = pi; % reference angular velocity
%% simulation parameter (To Create the Concept of Time)
total_time = 5;  % (seconds)
dt = 0.1; % sample sizes
time_stamp = total_time/dt; % total sample that we will see in simulation
i = 1; % start i from 1
%% initial and final conditions
x_pos = zeros(3, time_stamp); % to store each positon by each sample
point = [0;0;0]; % [x, y, theta]
position_r = [5;5;pi]; % target position and orientation
%% Motion
%% loop starts here - Robot move by time
figure(1)
scatter(position_r(1),position_r(2),200,"rx")
hold on
while i <= time_stamp
    x_pos(:,i) = point; % add the positions to the storage
    q_dot = model(theta_i,v_i,w_i); % kinematics calculation 
    point(1:2) = point(1:2) + q_dot(1:2) * dt; % move the robot's position
    theta_i = theta_i + q_dot(3) * dt; % move the robot's orientation
    point(3) = theta_i; % add theta value to the current configuration 
    scatter(x_pos(1,i),x_pos(2,i),"b*") % plot the position
    hold on % hold the previous graph
    pause(0.1) % delay
    e = error_function(position_r(1),position_r(2),position_r(3),point(1),point(2),theta_i); % error equations which describe the time evolution which is change of coordinates
    v_i = v_r * cos(e(3)) + 2 * e(1); % Check Canudas et al. (1994)
    w_i = w_r + (-2 * e(2) + (2*e(3))); % Check Canudas et al. (1994)
    er(:,i) = e; % Store the error values 
    time(i) = i/10; % store the each sample
    i = i + 1; % next iteration
end
grid on
xlabel('x,[m]'); ylabel('y,[m]');
title("Trajectory of the Robot",'FontSize',15)
legend('Target Position','Mobile Robot','Location',"southeast")
figure(2)
subplot(2,2,1);
plot(time,er(1,:),"r")
xlabel('Time'); ylabel('error');
title("Error in X Position")
subplot(2,2,2);
plot(time,er(2,:),"b")
xlabel('Time'); ylabel('error');
title("Error in Y Position")
subplot(2,2,3);
plot(x_pos(1,:),x_pos(2,:),"black")
xlabel('x,[m]'); ylabel('y,[m]');
title("Actual Trajectory")
subplot(2,2,4);
plot(time,x_pos(3,:),"green")
xlabel('Time'); ylabel('\theta');
title("Theta by Time")


