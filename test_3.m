clc 
clear all
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

x_desired = [1;1]

figure(1)
while i <= time_stamp
    x_pos(:,i) = point;
    error(i) = sqrt(((x_desired(1)-x_pos(1,i))^2)+((x_desired(2)-x_pos(2,i))^2))
    time(i) = i/10;
    q_dot = model(theta_i,v_i,w_i);
    point(1:2) = point(1:2) + q_dot(1:2) * dt;
    theta_i = theta_i + q_dot(3) * dt;
    scatter(x_pos(1,i),x_pos(2,i))
    hold on
    pause(1)
    disp(error)
    if i > 2
        if error(i) <= 0.1
            v_i = 0;
            w_i = 0;
        end
    end
    i = i + 1;
end
hold off
figure(2)
scatter(time,error(1,:))