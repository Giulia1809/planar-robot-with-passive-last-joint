clear;
clc;

% Execute code
zero_gravity_case;
%zero_gravity_off_path;
%nonzero_gravity_rest_to_rest;
%nonzero_gravity_swingup;

L1 = 1.5;
L2 = 1.5;
L3 = 1;
if psi == deg2rad(90)
    m1 = 10;
    m2 = 5;
    m3 = 1;
else
    m1 = 1;
    m2 = 1;
    m3 = 0.5;
end

rob= Robot(L1, L2, L3, m1, m2, m3, psi);

matrix = cell2mat(base_traj);
base_x = matrix(1,:);
base_y = matrix(2,:);
theta = matrix(3,:);

ee_x = base_x + l_3 *cos(theta);
ee_y = base_y + l_3 *sin(theta);

cp_x = base_x + K *cos(theta);
cp_y = base_y + K *sin(theta);

time = cell2mat(t_traj);

%% Folders creation
if not(exist("plots", 'dir'))
   mkdir plots;
   mkdir plots/nonzero_gravity/rest_to_rest;
   mkdir plots/nonzero_gravity/swingup;
   mkdir plots/zero_gravity;
end

if not(exist("plots/nonzero_gravity", 'dir'))
   mkdir plots/nonzero_gravity/rest_to_rest;
   mkdir plots/nonzero_gravity/swingup;
end

if not(exist("plots/zero_gravity", 'dir'))
   mkdir plots/zero_gravity;
end

%% Plot 1: link motion
clf;

daspect([1, 1, 1]);
if psi == deg2rad(0) && swingup
    xlim([-0.5, 2.5]);
    ylim([-0.5, 2.5]);
else
    xlim([0, 3]);
    ylim([0, 3]);
end
ylabel('y axis [m]');
xlabel('x axis [m]');
title('Cartesian motion of the third link');
grid on;
set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');

% base trajectory
line(base_x, base_y, 'LineWidth', 0.5, 'LineStyle',':', 'Color','black');

% ee_trajectory
line(ee_x, ee_y, 'LineWidth', 0.5, 'LineStyle','--', 'Color','black');

% initial state
line([base_x(1),  ee_x(1)], [base_y(1), ee_y(1)], 'LineWidth', 0.5, 'Color','black');
if psi == deg2rad(0)
    text( base_x(1) + 0.05, mean([base_y(1), ee_y(1)]), 'start', 'FontSize', 10);
elseif off_path
    text( mean([base_x(1), ee_x(1)]), base_y(1) + 0.05, 'start', 'FontSize', 10);
else
    text( mean([base_x(1), ee_x(1)]), base_y(1) + 0.1, 'start', 'FontSize', 10);
end

% final state
line([base_x(end), ee_x(end)], [base_y(end), ee_y(end)], 'LineWidth', 0.5, 'Color','black');
if psi == deg2rad(0)
    text( base_x(end) + 0.05, mean([base_y(end), ee_y(end)]), 'goal', 'FontSize', 10);
else
    text( mean([base_x(end), ee_x(end)]) + 0.05, mean([base_y(end), ee_y(end)])- 0.05 , 'goal', 'FontSize', 10);
end

if psi == deg2rad(0)
    if swingup
        saveas(gcf, 'plots/nonzero_gravity/swingup/link_traj.png');
    else
        if T == 1
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/link_traj_T_1.png');
        else
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/link_traj_T_5.png');
        end
    end
else
    if off_path
        saveas(gcf, 'plots/zero_gravity/link_traj_off_path.png');
    else
        saveas(gcf, 'plots/zero_gravity/link_traj.png');
    end
end

%% Plot 2: arm motion
clf;

if not(psi == deg2rad(0) && swingup)
    daspect([1, 1, 1]);
    xlim([-1, 3]);
    xticks(-1:0.5:3);
    ylim([-1, 3]);
    title('Arm motion');
    ylabel('y axis [m]');
    xlabel('x axis [m]');
    grid on;
    set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');

    M = fix(25*T);
    N = fix(size(ee_x,2)/M)+1;
    N_partial = fix(N/3);
    %ColorMap = [zeros(N,1), linspace(0,1,N).', ones(N,1)*0.6, [linspace(1,0.3,N_partial).'; ones(N-N_partial,1)*0.3]];
    ColorMap = [zeros(N,1), linspace(0,1,N).', [linspace(0.3,0.6,N_partial).'; ones(N-N_partial,1)*0.6], [linspace(1,0.3,2*N_partial).'; ones(N-2*N_partial,1)*0.3]];

    line([base_x(1), base_x(1)], [base_y(1), base_y(1)], 'Color', [0, 0, 0.3, 1] );
    line([base_x(size(ee_x,2)), base_x(size(ee_x,2))], [base_y(size(ee_x,2)), base_y(size(ee_x,2))], 'Color', [0, 1, 0.6, 1] );
    legend(["initial state", "final state"], 'location','southwest', 'Interpreter','latex');
    
    for i=1:M:size(ee_x,2)
        q = rob.InverseKinematics([base_x(i); base_y(i)]);

        % OBS base_x is the base of the third link
        line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix(i/M)+1,:), 'HandleVisibility','off');

        base_second_link = [base_x(i); base_y(i)] - [cos(q(1)+q(2)); sin(q(1)+q(2))] * L2;
        line([base_second_link(1), base_x(i)], [base_second_link(2), base_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix(i/M)+1,:), 'HandleVisibility','off');

        base_first_link = base_second_link - [cos(q(1)); sin(q(1))] * L1;
        line([base_first_link(1), base_second_link(1)], [base_first_link(2), base_second_link(2)], 'LineWidth', 0.5, 'Color', ColorMap(fix(i/M)+1,:), 'HandleVisibility','off');
    end 
    line(ee_x, ee_y, 'LineWidth', 0.5, 'LineStyle','--', 'Color','black', 'HandleVisibility','off');
    line(base_x, base_y, 'LineWidth', 0.5, 'LineStyle',':', 'Color','black', 'HandleVisibility','off');

    if psi == deg2rad(0)
        if T == 1
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/arm_traj_T_1.png');
        else
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/arm_traj_T_5.png');
        end
    else
        if off_path
            saveas(gcf, 'plots/zero_gravity/arm_traj_off_path.png');
        else
            saveas(gcf, 'plots/zero_gravity/arm_traj.png');
        end
    end

else
    % First phase
    f1 = figure(1);
    daspect([1, 1, 1]);
    xlim([-0.5, 2.5]);
    ylim([-0.5, 2.5]);
    grid on;
    set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');

    title('Cartesian motion of the third link: phase I');
    ylabel('y axis [m]');
    xlabel('x axis [m]');
    
    f2 = figure(2);
    daspect([1, 1, 1]);
    xlim([-0.5, 2.5]);
    ylim([-0.5, 2.5]);
    grid on;
    set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');

    title('Arm motion: phase I');
    ylabel('y axis [m]');
    xlabel('x axis [m]');
    
    M = fix(25*T_m);
    tot = find(time==T_m);
    N = fix(tot/M)+1;
    N_partial = fix(N/3);
    %ColorMap = [zeros(N,1), linspace(0,1,N).', ones(N,1)*0.6, [linspace(1,0.3,N_partial).'; ones(N-N_partial,1)*0.3]];
    ColorMap = [zeros(N,1), linspace(0,1,N).', [linspace(0.3,0.6,N_partial).'; ones(N-N_partial,1)*0.6], [linspace(1,0.3,2*N_partial).'; ones(N-2*N_partial,1)*0.3]];
    
    figure(1);
    line([base_x(1), base_x(1)], [base_y(1), base_y(1)], 'Color', [0, 0, 0.3, 1] );
    line([base_x(tot), base_x(tot)], [base_y(tot), base_y(tot)], 'Color', [0, 1, 0.6, 1] );
    legend(["initial state", "final state"], 'location','northeast', 'Interpreter','latex');
    
    figure(2);
    line([base_x(1), base_x(1)], [base_y(1), base_y(1)], 'Color', [0, 0, 0.3, 1] );
    line([base_x(tot), base_x(tot)], [base_y(tot), base_y(tot)], 'Color', [0, 1, 0.6, 1] );
    legend(["initial state", "final state"], 'location','northeast', 'Interpreter','latex');
    
    for i=1:M:tot
        q = rob.InverseKinematics([base_x(i); base_y(i)]);

        figure(1);
        % OBS base_x is the base of the third link
        line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix(i/M)+1,:), 'HandleVisibility','off');   
        
        figure(2);
        line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix(i/M)+1,:), 'HandleVisibility','off');

        base_second_link = [base_x(i); base_y(i)] - [cos(q(1)+q(2)); sin(q(1)+q(2))] * L2;
        line([base_second_link(1), base_x(i)], [base_second_link(2), base_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix(i/M)+1,:), 'HandleVisibility','off');

        base_first_link = base_second_link - [cos(q(1)); sin(q(1))] * L1;
        line([base_first_link(1), base_second_link(1)], [base_first_link(2), base_second_link(2)], 'LineWidth', 0.5, 'Color', ColorMap(fix(i/M)+1,:), 'HandleVisibility','off');
    end 
    
    figure(1);
    line(ee_x(1:i), ee_y(1:i), 'LineWidth', 0.5, 'LineStyle','--', 'Color','black', 'HandleVisibility','off');
    line(base_x(1:i), base_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color','black', 'HandleVisibility','off');
    
    saveas(f1, 'plots/nonzero_gravity/swingup/link_traj_I.png');   
    
    figure(2);
    line(ee_x(1:i), ee_y(1:i), 'LineWidth', 0.5, 'LineStyle','--', 'Color','black', 'HandleVisibility','off');
    line(base_x(1:i), base_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color','black', 'HandleVisibility','off');
    
    saveas(f2, 'plots/nonzero_gravity/swingup/arm_traj_I.png');
    
    % Second phase
    f1 = figure(1);
    clf;
    daspect([1, 1, 1]);
    xlim([-0.5, 2.5]);
    ylim([-0.5, 2.5]);
    grid on;
    set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');

    title('Cartesian motion of the third link: phase II');
    ylabel('y axis [m]');
    xlabel('x axis [m]');
    
    f2 = figure(2);
    clf;
    daspect([1, 1, 1]);
    xlim([-0.5, 2.5]);
    ylim([-0.5, 2.5]);
    grid on;
    set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');

    title('Arm motion: phase II');
    ylabel('y axis [m]');
    xlabel('x axis [m]');
    
    M = fix(25*(T));
    tot = size(ee_x,2) - tot;
    N = fix(tot/M)+1;
    N_partial = fix(N/3);
    initial_i = i;
    %ColorMap = [zeros(N,1), linspace(0,1,N).', ones(N,1)*0.6, [linspace(1,0.3,N_partial).'; ones(N-N_partial,1)*0.3]];
    ColorMap = [zeros(N,1), linspace(0,1,N).', [linspace(0.3,0.6,N_partial).'; ones(N-N_partial,1)*0.6], [linspace(1,0.3,2*N_partial).'; ones(N-2*N_partial,1)*0.3]];

    line([base_x(initial_i), base_x(initial_i)], [base_y(initial_i), base_y(initial_i)], 'Color', [0, 0, 0.3, 1] );
    line([base_x(size(ee_x,2)), base_x(size(ee_x,2))], [base_y(size(ee_x,2)), base_y(size(ee_x,2))], 'Color', [0, 1, 0.6, 1] );
    legend(["initial state", "final state"], 'location','northeast', 'Interpreter','latex');
    
    for i=initial_i:M:size(ee_x,2)
        q = rob.InverseKinematics([base_x(i); base_y(i)]);

        figure(1);
        % OBS base_x is the base of the third link
        line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix((i-initial_i)/M)+1,:), 'HandleVisibility','off');
 
        figure(2);
        line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix((i-initial_i)/M)+1,:), 'HandleVisibility','off');

        base_second_link = [base_x(i); base_y(i)] - [cos(q(1)+q(2)); sin(q(1)+q(2))] * L2;
        line([base_second_link(1), base_x(i)], [base_second_link(2), base_y(i)], 'LineWidth', 0.5, 'Color', ColorMap(fix((i-initial_i)/M)+1,:), 'HandleVisibility','off');

        base_first_link = base_second_link - [cos(q(1)); sin(q(1))] * L1;
        line([base_first_link(1), base_second_link(1)], [base_first_link(2), base_second_link(2)], 'LineWidth', 0.5, 'Color', ColorMap(fix((i-initial_i)/M)+1,:), 'HandleVisibility','off');
    end 

    figure(1);
    line(ee_x(initial_i:end), ee_y(initial_i:end), 'LineWidth', 0.5, 'LineStyle','--', 'Color','black', 'HandleVisibility','off');
    line(base_x(initial_i:end), base_y(initial_i:end), 'LineWidth', 0.5, 'LineStyle',':', 'Color','black', 'HandleVisibility','off');
    
    saveas(f1, 'plots/nonzero_gravity/swingup/link_traj_II.png');
    
    figure(2);
    line(ee_x(initial_i:end), ee_y(initial_i:end), 'LineWidth', 0.5, 'LineStyle','--', 'Color','black', 'HandleVisibility','off');
    line(base_x(initial_i:end), base_y(initial_i:end), 'LineWidth', 0.5, 'LineStyle',':', 'Color','black', 'HandleVisibility','off');
    
    saveas(f2, 'plots/nonzero_gravity/swingup/arm_traj_II.png');
end

%% Plot 3: joint torques 
clf;

matrix = cell2mat(base_vel);
base_x_d = matrix(1,:);
base_y_d = matrix(2,:);
theta_d = matrix(3,:);

matrix = cell2mat(base_acc);
a_x = matrix(1,:);
a_y = matrix(2,:);

title('Joint torques');
ylabel('Torque [Nm]');
xlabel('Time [s]');

if psi == deg2rad(0)
    if swingup
        ylim([-300, 600]);
    else
        if T == 1
            ylim([-200, 700]);
            xticks(0:0.1:T);
        else
            ylim([-20, 40]);
            xticks(0:0.5:T);
        end
    end
else
    if not(off_path)
        ylim([-60, 120]);
        xlim([0,10]);
        xticks(0:1:10);
    end
end

grid on;
set(gca,'gridLineStyle',':', 'gridAlpha', 0.8, 'gridColor','black');

tau_prev= rob.ForceTransformation([base_x(1); base_y(1)], theta(1), [base_x_d(1); base_y_d(1)], theta_d(1), [a_x(1); a_y(1)]);

for i=2:size(time, 2)      
    
    tau = rob.ForceTransformation([base_x(i); base_y(i)], theta(i), [base_x_d(i); base_y_d(i)], theta_d(i), [a_x(i); a_y(i)]);
    
    line([time(i)-step, time(i)], [tau_prev(1), tau(1)], 'LineWidth', 0.5, 'Color', [1, 0.375, 0, 1]);
    line([time(i)-step, time(i)], [tau_prev(2), tau(2)], 'LineWidth', 0.5, 'Color', [0.0 0.719 1, 1]);
    
    tau_prev= tau;
end

legend(["$\tau_1$", "$\tau_2$"], 'location','best', 'Interpreter','latex');

if psi == deg2rad(0)
    if swingup
        saveas(gcf, 'plots/nonzero_gravity/swingup/tau_traj.png');
    else
        if T == 1
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/tau_traj_T_1.png');
        else
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/tau_traj_T_5.png');
        end
    end
else
    if off_path
        saveas(gcf, 'plots/zero_gravity/tau_traj_off_path.png');
    else
        saveas(gcf, 'plots/zero_gravity/tau_traj.png');
    end
end

%% Plot 4: output errors 
clf;

title('Output errors');
ylabel('Error');
xlabel('Time [s]');
ylim([-0.08, 0.04]);
if off_path
    xticks(0:1:10);
end
grid on;
set(gca,'gridLineStyle',':', 'gridAlpha', 0.8, 'gridColor','black');

error_y_traj = cell2mat(error_traj);
linestyle = ['-', ':'];

% error_y(i, :) trajectory
for i=1:2
    line(time(2:end), error_y_traj(i, 2:end), 'LineWidth', 0.5, 'LineStyle', linestyle(i), 'Color','black');
end

legend(["$e_1$", "$e_2$"], 'location','best', 'Interpreter','latex');

if psi == deg2rad(0)
    if swingup
        saveas(gcf, 'plots/nonzero_gravity/swingup/error_traj.png');
    else
        if T == 1
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/error_traj_T_1.png');
        else
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/error_traj_T_5.png');
        end
    end
else
    if off_path
        saveas(gcf, 'plots/zero_gravity/error_traj_off_path.png');
    else
        saveas(gcf, 'plots/zero_gravity/error_traj.png');
    end
end

%% Plot 5: rho 
clf;

title('Rho evolution');
xlabel('Time [s]');
grid on;
set(gca,'gridLineStyle',':', 'gridAlpha', 0.8, 'gridColor','black');

if psi == deg2rad(0)
    g0 = 0;
else
    g0 = 9.81 * cos(psi);
end

xi = cell2mat(xi_traj);

rho_prev = xi(1) + g0 * sin(theta(1));

for i=2:size(time, 2)      
    
    rho = xi(i) + g0 * sin(theta(i));
    
    line([time(i)-step, time(i)], [rho_prev, rho], 'LineWidth', 0.5, 'Color','black');
        
    rho_prev= rho;
end

if psi == deg2rad(0)
    if swingup
        saveas(gcf, 'plots/nonzero_gravity/swingup/rho_traj.png');
    else
        if T == 1
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/rho_traj_T_1.png');
        else
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/rho_traj_T_5.png');
        end
    end
else
    if off_path
        saveas(gcf, 'plots/zero_gravity/rho_traj_off_path.png');
    else
        saveas(gcf, 'plots/zero_gravity/rho_traj.png');
    end
end

%% Plot 6: q plot

clf;

title('Joint evolutions');
ylabel('Angle [rad]');
xlabel('Time [s]');
grid on;
set(gca,'gridLineStyle',':', 'gridAlpha', 0.8, 'gridColor','black');

q = rob.InverseKinematics([base_x(1); base_y(1)]);
q_prev= [q(1); q(2); wrapToPi(theta(1)-q(1)-q(2))];

if psi == deg2rad(0) && swingup
    M = fix(T_m+T);
else
    M = fix(T);
end

for i=(1+M):M:size(time, 2)     
    
    q = rob.InverseKinematics([base_x(i); base_y(i)]);
    q= [q(1); q(2); wrapToPi(theta(i)-q(1)-q(2))];
    
    line([time(i)-M*step, time(i)], [q_prev(1), q(1)], 'LineWidth', 0.5, 'Color', "#FF6347");
    line([time(i)-M*step, time(i)], [q_prev(2), q(2)], 'LineWidth', 0.5, 'Color', "#FF8C00");
    line([time(i)-M*step, time(i)], [q_prev(3), q(3)], 'LineWidth', 0.5, 'Color', "#3CB371");
        
    q_prev= q;
end

legend(["$q_1$", "$q_2$", "$q_3$"], 'location','best', 'Interpreter','latex');

if psi == deg2rad(0)
    if swingup
        saveas(gcf, 'plots/nonzero_gravity/swingup/q_traj.png');
    else
        if T == 1
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/q_traj_T_1.png');
        else
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/q_traj_T_5.png');
        end
    end
else
    if off_path
        saveas(gcf, 'plots/zero_gravity/q_traj_off_path.png');
    else
        saveas(gcf, 'plots/zero_gravity/q_traj.png');
    end
end

%% Plot 7: q_d plot

clf;

title('Joint velocity evolutions');
ylabel('Velocity [rad/s]');
xlabel('Time [s]');
grid on;
set(gca,'gridLineStyle',':', 'gridAlpha', 0.8, 'gridColor','black');
            
q = rob.InverseKinematics([base_x(1); base_y(1)]);
q = [q(1); q(2); wrapToPi(theta(1)-q(1)-q(2))];

J = [-rob.L2*sin(q(1) + q(2)) - rob.L1*sin(q(1)), -rob.L2*sin(q(1) + q(2));
      rob.L2*cos(q(1) + q(2)) + rob.L1*cos(q(1)),  rob.L2*cos(q(1) + q(2))];
p_d = base_vel{1};
q_d = pinv(J)*p_d(1:2);
q_d_prev = [q_d; p_d(3) - q_d(1) - q_d(2)];

if psi == deg2rad(0) && swingup
    M = fix(T_m+T);
else
    M = fix(T);
end

for i=(1+M):M:size(time, 2)    
    
    q = rob.InverseKinematics([base_x(i); base_y(i)]);
    q = [q(1); q(2); wrapToPi(theta(i)-q(1)-q(2))];
    J = [-rob.L2*sin(q(1) + q(2)) - rob.L1*sin(q(1)), -rob.L2*sin(q(1) + q(2));
          rob.L2*cos(q(1) + q(2)) + rob.L1*cos(q(1)),  rob.L2*cos(q(1) + q(2))];

    p_d = base_vel{i};
    q_d = pinv(J)*p_d(1:2);
    q_d = [q_d; p_d(3) - q_d(1) - q_d(2)];

    line([time(i)-M*step, time(i)], [q_d_prev(1), q_d(1)], 'LineWidth', 0.5, 'Color', "#FF6347");
    line([time(i)-M*step, time(i)], [q_d_prev(2), q_d(2)], 'LineWidth', 0.5, 'Color', "#FF8C00");
    line([time(i)-M*step, time(i)], [q_d_prev(3), q_d(3)], 'LineWidth', 0.5, 'Color', "#3CB371");
        
    q_d_prev= q_d;
end

legend(["$\dot{q}_1$", "$\dot{q}_2$", "$\dot{q}_3$"], 'location','best', 'Interpreter','latex');

if psi == deg2rad(0)
    if swingup
        saveas(gcf, 'plots/nonzero_gravity/swingup/q_d_traj.png');
    else
        if T == 1
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/q_d_traj_T_1.png');
        else
            saveas(gcf, 'plots/nonzero_gravity/rest_to_rest/q_d_traj_T_5.png');
        end
    end
else
    if off_path
        saveas(gcf, 'plots/zero_gravity/q_d_traj_off_path.png');
    else
        saveas(gcf, 'plots/zero_gravity/q_d_traj.png');
    end
end

%% Video: arm evolution
clf;

if not(psi == deg2rad(0) && swingup)
    daspect([1, 1, 1]);
    xlim([-1, 3]);
    ylim([-1, 3]);
    title('arm motion');
    ylabel('y axis [m]');
    xlabel('x axis [m]');
    grid on;
    set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');

    waitforbuttonpress;
    hold on;
    
    if T == 1
        M = 5;
    else
        M = 15;
    end
    for i=1:M:size(ee_x,2)
        
        q = rob.InverseKinematics([base_x(i); base_y(i)]);

        % OBS base_x is the base of the third link
        line1 = line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', 'black', 'HandleVisibility','off');

        base_second_link = [base_x(i); base_y(i)] - [cos(q(1)+q(2)); sin(q(1)+q(2))] * L2;
        line2 = line([base_second_link(1), base_x(i)], [base_second_link(2), base_y(i)], 'LineWidth', 0.5, 'Color','black', 'HandleVisibility','off');

        base_first_link = base_second_link - [cos(q(1)); sin(q(1))] * L1;
        line3 = line([base_first_link(1), base_second_link(1)], [base_first_link(2), base_second_link(2)], 'LineWidth', 0.5, 'Color', 'black', 'HandleVisibility','off');
        
        line(ee_x(1:i), ee_y(1:i), 'LineWidth', 0.5, 'LineStyle','--', 'Color','black', 'HandleVisibility','off');
        line(base_x(1:i), base_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color','black', 'HandleVisibility','off');
        line(cp_x(1:i), cp_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color','red', 'HandleVisibility','off');
        
        drawnow;
        
        if (i + M >= size(ee_x,2))
        elseif not(i==1)
            delete(line1);
            delete(line2);
            delete(line3);
        end
    end 

else
    % First phase
    daspect([1, 1, 1]);
    xlim([-0.5, 2.5]);
    ylim([-0.5, 2.5]);
    title('arm motion: phase I');
    ylabel('y axis [m]');
    xlabel('x axis [m]');
    grid on;
    set(gca,'gridLineStyle',':', 'gridAlpha', 0.5, 'gridColor','black');
    
    tot = find(time==T_m);
    
    waitforbuttonpress;
    hold on;
    
    M = 5;
    for i=1:M:tot
        q = rob.InverseKinematics([base_x(i); base_y(i)]);

        line1 = line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', 'black');

        base_second_link = [base_x(i); base_y(i)] - [cos(q(1)+q(2)); sin(q(1)+q(2))] * L2;
        line2 = line([base_second_link(1), base_x(i)], [base_second_link(2), base_y(i)], 'LineWidth', 0.5, 'Color', 'black');

        base_first_link = base_second_link - [cos(q(1)); sin(q(1))] * L1;
        line3 = line([base_first_link(1), base_second_link(1)], [base_first_link(2), base_second_link(2)], 'LineWidth', 0.5, 'Color', 'black');
        
        line4 = line(ee_x(1:i), ee_y(1:i), 'LineWidth', 0.5, 'LineStyle','--', 'Color', 'black');
        line5 = line(base_x(1:i), base_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color', 'black');
        line6 = line(cp_x(1:i), cp_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color', 'red');
        
        drawnow;
        
        if i==1
            ln1=line1;
            ln2=line2;
            ln3=line3;
        elseif not(i==tot)
            delete(line1);
            delete(line2);
            delete(line3);
            delete(line4);
            delete(line5);
            delete(line6);
        end
    end 
 
    % Second phase
   
    waitforbuttonpress;
    hold on;
    
    delete(ln1);
    delete(ln2);
    delete(ln3);
    delete(line4);
    delete(line5);
    delete(line6);
   
    title('arm motion: phase II');
    
    initial_i = i;
    
    M = 5;
    for i=initial_i:M:size(ee_x,2)
        q = rob.InverseKinematics([base_x(i); base_y(i)]);
        
        line1 = line([base_x(i), ee_x(i)], [base_y(i), ee_y(i)], 'LineWidth', 0.5, 'Color', 'black');

        base_second_link = [base_x(i); base_y(i)] - [cos(q(1)+q(2)); sin(q(1)+q(2))] * L2;
        line2 = line([base_second_link(1), base_x(i)], [base_second_link(2), base_y(i)], 'LineWidth', 0.5, 'Color', 'black');

        base_first_link = base_second_link - [cos(q(1)); sin(q(1))] * L1;
        line3 = line([base_first_link(1), base_second_link(1)], [base_first_link(2), base_second_link(2)], 'LineWidth', 0.5, 'Color', 'black');
    
        line4 = line(ee_x(1:i), ee_y(1:i), 'LineWidth', 0.5, 'LineStyle','--', 'Color', 'black');
        line5 = line(base_x(1:i), base_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color', 'black');
        line6 = line(cp_x(1:i), cp_y(1:i), 'LineWidth', 0.5, 'LineStyle',':', 'Color', 'red');
        
        drawnow;
        
        if not(i + M >= size(ee_x,2))
            delete(line1);
            delete(line2);
            delete(line3);
        end
    end

end