clear;
clc;

% swing up
q_s = [0.75; 1; deg2rad(-90)];
q_s_d = zeros(3, 1);
xi_s = 0;
eta_s = 0;

q_g = [0.75; 1; deg2rad(90)];
q_g_d = zeros(3, 1);
xi_g = -20;
eta_g = 0;

q_m = [0.75; 1; deg2rad(0)];
q_m_d = zeros(3, 1);
xi_m = -10;
eta_m = 0;

T_m = 1;
T = 1.6 - T_m;
step = 0.001;
l_3 = 1;
K = 2/3 * l_3;
psi = deg2rad(0);

F = [8; 24; 32; 16];

robot = PFLSystem(psi, l_3, q_s, q_s_d);

dyn_comp = DynamicCompensator(robot, eta_s, xi_s);

%boundary conditions [y_s; y_s_d; y_s_d_d; y_s_3_d]
y_1_s = [ q_s(1);
               0;
               0;
               0];
y_1_g = [ q_g(1);
               0;
               0;
               0];

y_2_s = [ q_s(2) - K;
                   0;
               -xi_s;
              -eta_s];
y_2_g = [ q_g(2) + K;
                   0;
                xi_g;
               eta_g];
           
y_1_m = [                  q_m(1) + K;
                             q_m_d(1);
                                 xi_m;
          eta_m + robot.g0 * q_m_d(3)];
y_2_m = [                      q_m(2);
              q_m_d(2) + K * q_m_d(3);
                            -robot.g0;
                      xi_m * q_m_d(3)];

traj_y_1_first_p = trajectoryGeneration(y_1_s, y_1_m, T_m);
traj_y_2_first_p = trajectoryGeneration(y_2_s, y_2_m, T_m);

traj_y_1_second_p = trajectoryGeneration(y_1_m, y_1_g, T);
traj_y_2_second_p = trajectoryGeneration(y_2_m, y_2_g, T);

% initialize plots info
swingup = true;

base_traj = { q_s };
base_vel = {[0 ; 0; 0]};
base_acc = {[0; 0]};
xi_traj = {xi_s};

y_1_des = traj_y_1_first_p(0);
y_2_des = traj_y_2_first_p(0);

y = dyn_comp.y;
y_des = [y_1_des(4); y_2_des(4)];
error_y = y_des - y;

error_traj = {error_y};

t_traj = {0};

for elapse_time = step:step:T_m
    display("elapse_time = " + string(elapse_time));
    y_1_des = traj_y_1_first_p(elapse_time);
    y_2_des = traj_y_2_first_p(elapse_time);
    
    y = [dyn_comp.y_3_d'; dyn_comp.y_d_d'; dyn_comp.y_d'; dyn_comp.y'];
    
    y_des = [flip(y_1_des(1:4)), flip(y_2_des(1:4))];
    
    %error
    error_y = y_des - y;
    
    v = [controller(y_1_des(5), error_y(:,1), F); controller(y_2_des(5), error_y(:,2), F)];
    %v = [y_1_des(5); y_2_des(5)]
    
    %apply control
    dyn_comp.Integrate(v, step);
    
    % update plot info
    base_traj{end+1} = dyn_comp.pfl_robot.q;
    base_vel{end+1} = dyn_comp.pfl_robot.q_d;
    base_acc{end+1} = dyn_comp.a;
    xi_traj{end+1} = dyn_comp.xi;
    error_traj{end+1} = [error_y(4,1); error_y(4,2)];   
    t_traj{end+1} = elapse_time;
end

display("FINE PRIMA FASE");

for elapse_time = step:step:T
    display("elapse_time = " + string(T_m + elapse_time));
    y_1_des = traj_y_1_second_p(elapse_time);
    y_2_des = traj_y_2_second_p(elapse_time);
    
    y = [dyn_comp.y_3_d'; dyn_comp.y_d_d'; dyn_comp.y_d'; dyn_comp.y'];
    
    y_des = [flip(y_1_des(1:4)), flip(y_2_des(1:4))];
    
    %error
    error_y = y_des - y;
    
    v = [controller(y_1_des(5), error_y(:,1), F); controller(y_2_des(5), error_y(:,2), F)];
    %v = [y_1_des(5); y_2_des(5)]
    
    %apply control
    dyn_comp.Integrate(v, step);
    
    % update plot info
    base_traj{end+1} = dyn_comp.pfl_robot.q;
    base_vel{end+1} = dyn_comp.pfl_robot.q_d;
    base_acc{end+1} = dyn_comp.a;
    xi_traj{end+1} = dyn_comp.xi;
    error_traj{end+1} = [error_y(4,1); error_y(4,2)];   
    t_traj{end+1} = elapse_time + T_m;
end

display(dyn_comp.pfl_robot.q);

