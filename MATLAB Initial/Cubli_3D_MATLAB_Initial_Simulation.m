clear; clc; close all;

%% System Parameters

safety_factor = 1.5;
theta0_deg = 0.1;
theta_end_deg = 5;
g = 9.81;

m = 1.762772;
r_com = [0.070565; 0.070496; 0.070498];

I_cubli = [2.266e-02, -8.953e-03, -8.953e-03;
         -8.953e-03,  2.268e-02, -8.945e-03;
         -8.953e-03, -8.945e-03,  2.268e-02];

I_w_full = [1.914e-04, -2.200e-11, -4.000e-12;
            -2.200e-11,  1.914e-04,  2.000e-12;
            -4.000e-12,  2.000e-12,  3.822e-04];

[~, I_w_diagonal] = eig(I_w_full);
spinning_axis = max(diag(I_w_diagonal));
I_w     = spinning_axis * eye(3);
I_bar_c = I_cubli - I_w;

%% Equilibrium State Definition

com_dir   = r_com / norm(r_com);
global_up = [0; 0; 1];

rot_axis  = cross(com_dir, global_up);
rot_axis  = rot_axis / norm(rot_axis);
rot_angle = acos(dot(com_dir, global_up));

q0 = [cos(rot_angle/2); sin(rot_angle/2) * rot_axis];
if isrow(q0), q0 = q0'; end 

syms q1 q2 q3 q4 omega_c1 omega_c2 omega_c3 omega_w1 omega_w2 omega_w3
syms u1 u2 u3
q   = [q1; q2; q3; q4]; 
omega_c = [omega_c1; omega_c2; omega_c3];
omega_w = [omega_w1; omega_w2; omega_w3];
x_sym = [q; omega_c; omega_w];
u_sym = [u1; u2; u3];
%% Symbolic Dynamics

% Rotation Matrix from Body to Global
R_B2G = [ 1-2*(x_sym(3)^2+x_sym(4)^2),      2*(x_sym(2)*x_sym(3) - x_sym(1)*x_sym(4)), 2*(x_sym(2)*x_sym(4) + x_sym(1)*x_sym(3));
          2*(x_sym(2)*x_sym(3) + x_sym(1)*x_sym(4)), 1-2*(x_sym(2)^2+x_sym(4)^2),      2*(x_sym(3)*x_sym(4) - x_sym(1)*x_sym(2));
          2*(x_sym(2)*x_sym(4) - x_sym(1)*x_sym(3)), 2*(x_sym(3)*x_sym(4) + x_sym(1)*x_sym(2)), 1-2*(x_sym(2)^2+x_sym(3)^2) ];

g_body = R_B2G' * [0; 0; -g];
tau_g = cross(r_com, m * g_body);

H_total = (I_bar_c * omega_c) + (I_w * omega_w);
rhs_c   = -cross(omega_c, H_total) + tau_g - u_sym;
omega_dot_c = I_bar_c \ rhs_c;

rhs_w   = u_sym;
omega_dot_w = (I_w \ rhs_w) - omega_dot_c;

Omega = [ 0,         -x_sym(5), -x_sym(6), -x_sym(7);
          x_sym(5),   0,         x_sym(7), -x_sym(6);
          x_sym(6),  -x_sym(7),  0,         x_sym(5);
          x_sym(7),   x_sym(6), -x_sym(5),  0];
dqdt = 0.5 * Omega * q;

dxdt_sym = [dqdt; omega_dot_c; omega_dot_w];

dyn_func = matlabFunction(dxdt_sym, 'Vars', {x_sym, u_sym});

%% Fall Simulation Setup

tilt_angle_rad  = deg2rad(theta0_deg);
q_tilt_start    = [cos(tilt_angle_rad/2), sin(tilt_angle_rad/2), 0, 0];

% Apply tilt to balanced state
q_start = [q_tilt_start(1)*q0(1) - q_tilt_start(2)*q0(2) - q_tilt_start(3)*q0(3) - q_tilt_start(4)*q0(4);
           q_tilt_start(1)*q0(2) + q_tilt_start(2)*q0(1) + q_tilt_start(3)*q0(4) - q_tilt_start(4)*q0(3);
           q_tilt_start(1)*q0(3) - q_tilt_start(2)*q0(4) + q_tilt_start(3)*q0(1) + q_tilt_start(4)*q0(2);
           q_tilt_start(1)*q0(4) + q_tilt_start(2)*q0(3) - q_tilt_start(3)*q0(2) + q_tilt_start(4)*q0(1)];

x_start = [q_start; 0; 0; 0; 0; 0; 0];
u_free_fall = [0; 0; 0];
theta_stop_rad = deg2rad(theta_end_deg);
tspan = [0 10];

opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'Events', @(t,x) fall_event_3d(t, x, theta_stop_rad, r_com));
[t_sim, x_sim] = ode45(@(t,x) dyn_func(x, u_free_fall), tspan, x_start, opts);

%% Kinematics & Required Effort

N = size(x_sim, 1);
theta_sim = zeros(N, 1);
tau_g_sim = zeros(N, 3);
omega_global_sim = zeros(N, 3);
omega_body_sim = x_sim(:, 5:7);

for k = 1:N
    qk = x_sim(k, 1:4)';
    qk = qk / norm(qk); 
    
    % R_B2G matrix evaluation
    R_mat = [ 1-2*(qk(3)^2+qk(4)^2),      2*(qk(2)*qk(3)-qk(1)*qk(4)), 2*(qk(2)*qk(4)+qk(1)*qk(3));
              2*(qk(2)*qk(3)+qk(1)*qk(4)), 1-2*(qk(2)^2+qk(4)^2),      2*(qk(3)*qk(4)-qk(1)*qk(2));
              2*(qk(2)*qk(4)-qk(1)*qk(3)), 2*(qk(3)*qk(4)+qk(1)*qk(2)), 1-2*(qk(2)^2+qk(3)^2) ];
          
    R_G2B = R_mat';

    g_body_val = R_G2B * [0; 0; -g];
    tau_g_sim(k, :) = cross(r_com, m * g_body_val)';

    omega_global_sim(k, :) = (R_mat * omega_body_sim(k, :)')';
    
    r_com_global = R_mat * r_com;
    theta_sim(k) = atan2(norm(r_com_global(1:2)), r_com_global(3));
end

L_absorbed_body = cumtrapz(t_sim, tau_g_sim);
omega_w_req_body = L_absorbed_body ./ spinning_axis;
rpm_req_body_sim = omega_w_req_body * (60 / (2*pi));

max_omega_global = max(abs(omega_global_sim)) * (180/pi);
max_tau_req_body = max(abs(tau_g_sim));
max_rpm_req_body = max(abs(rpm_req_body_sim));

t_fall = t_sim(end) - t_sim(1);
omega_dot_w_req_body = max(abs(omega_w_req_body)) / t_fall;
tau_dynamic_body = spinning_axis .* omega_dot_w_req_body;

total_tau_req_body = max_tau_req_body + tau_dynamic_body;
max_total_tau = max(total_tau_req_body * safety_factor);

%% Terminal Printout

fprintf('\n==========================================================\n');
fprintf('%-45s %.1f -> %.1f deg\n', 'Stability Window:', theta0_deg, theta_end_deg);
fprintf('----------------------------------------------------------\n');
fprintf('MINIMUM REQUIRED ANGULAR VELOCITY [deg/s]\n');
fprintf('  [%.4f, %.4f, %.4f]\n', max_omega_global);
fprintf('SF REQUIRED ANGULAR VELOCITY [deg/s]\n');
fprintf('  %.4f \n', max(max_omega_global*safety_factor));
fprintf('----------------------------------------------------------\n');
fprintf('MINIMUM REQUIRED MOTOR TORQUE [N*m]\n');
fprintf('  [ %.4f,  %.4f,  %.4f]\n', total_tau_req_body);
fprintf('SF REQUIRED MOTOR TORQUE [N*m] \n');
fprintf('   %.4f\n', max_total_tau);
fprintf('--------------------------------------------------\n');
fprintf('MINIMUM REQUIRED REACTION WHEEL RPM [RPM]\n');
fprintf('  [%.1f, %.1f, %.1f]\n', max_rpm_req_body);
fprintf('SF REQUIRED REACTION WHEEL RPM [RPM]\n');
fprintf('  %.1f \n', max(max_rpm_req_body*safety_factor));
fprintf('==========================================================\n\n');

%% Plotting

figure('Name', '3D Cubli Free Fall Tilt Simulation');
theta_sim_d = rad2deg(theta_sim);
plot(t_sim, theta_sim_d, 'b', 'LineWidth', 2);
grid on;
title('3D True Tilt Error During Fall');
xlabel('Time (s)');
ylabel('Angle (deg)');
xlim([0, t_sim(end)*1.1]);

%% Helper Function

function [value, isterminal, direction] = fall_event_3d(~, x, theta_stop_rad, r_com)

    q = x(1:4)' / norm(x(1:4));
    R_B2G = [ 1-2*(q(3)^2+q(4)^2),      2*(q(2)*q(3)-q(1)*q(4)), 2*(q(2)*q(4)+q(1)*q(3));
              2*(q(2)*q(3)+q(1)*q(4)), 1-2*(q(2)^2+q(4)^2),      2*(q(3)*q(4)-q(1)*q(2));
              2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4)+q(1)*q(2)), 1-2*(q(2)^2+q(3)^2) ];
          
    r_com_global = R_B2G * r_com;
    
    current_tilt = atan2(norm(r_com_global(1:2)), r_com_global(3));
    
    value = current_tilt - theta_stop_rad;
    isterminal = 1;
    direction = 1;
end