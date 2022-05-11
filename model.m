%% Model of the System

function q_dot = model(theta_i,v_i,w_i)
T = [cos(theta_i) 0;
    sin(theta_i) 0;
    0 1];
u = [v_i; w_i];
q_dot = T * u;
end






