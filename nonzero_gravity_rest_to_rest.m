clear;
clc;

% rest to rest
q_s = [0.75; 1; deg2rad(90)];
q_s_d = zeros(3, 1);
xi_s = 0;
eta_s = 0;

q_g = [1.75; 1; deg2rad(90)];
q_g_d = zeros(3, 1);
xi_g = 0;
eta_g = 0;

T = 1;
%T = 5;
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

y_2_s = [ q_s(2) + K;
                   0;
                xi_s;
               eta_s];
y_2_g = [ q_g(2) + K;
                   0;
                xi_g;
               eta_g];

traj_y_1 = trajectoryGeneration(y_1_s, y_1_g, T);
traj_y_2 = trajectoryGeneration(y_2_s, y_2_g, T);

% initialize plots info
swingup = false;

base_traj = { q_s };
base_vel = {[0 ; 0; 0]};
base_acc = {[0; 0]};
xi_traj = {xi_s};

y_1_des = traj_y_1(0);
y_2_des = traj_y_2(0);

y = dyn_comp.y;
y_des = [y_1_des(4); y_2_des(4)];
error_y = y_des - y;

error_traj = {error_y};

t_traj = {0};

for elapse_time = step:step:T
    display(elapse_time);
    y_1_des = traj_y_1(elapse_time);
    y_2_des = traj_y_2(elapse_time);
    
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

display(dyn_comp.pfl_robot.q);


