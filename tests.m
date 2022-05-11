clc 
clear all
%% Parameters
% theta_i = 0;
v_i = 1;
w_i = 1;

total_time = 50  % (seconds)
dt = 0.5 % sample sizes
time_stamp = total_time/dt
i = 1
%% Robot 1
x_pos_1 = zeros(3, time_stamp);
point_1 = [0;0;0];

%% Robot 2
x_pos_2 = zeros(3, time_stamp);
point_2 = [1;1;0];

%% Robot 3
x_pos_3 = zeros(3, time_stamp);
point_3 = [2;2;0];

%% Combine Robot Parameters
x_pos(:,:,1) = x_pos_1
x_pos(:,:,2) = x_pos_2
x_pos(:,:,3) = x_pos_3

point(:,:,1) = point_1
point(:,:,2) = point_2
point(:,:,3) = point_3
%% Motion
while i <= time_stamp
    for j = 1:3
        x_pos(:,i,j) = point(:,:,j);
        q_dot(:,:,j) = model(point(3,j),v_i,w_i);
        point(1:2,j) = point(1:2,j) + q_dot(1:2,j) * dt;
        point(3,j) = point(3,j) + q_dot(3,j) * dt;
        scatter(x_pos(1,i,j),x_pos(2,i,j))
        hold on
        pause(0.05)
        %disp(point)
    end
    i = i + 1;
end