clear; clc; close all;

%% System Parameters

safety_factor = 1.5;
theta0_deg = 0.1;
theta_end_deg = 5;
g = 9.81;

m = 1.762772;
r_com = [0.070565; 0.070496; 0.070498];
l = sqrt(r_com(2)^2 + r_com(3)^2);

I_cubli =    [2.266e-02, -8.953e-03, -8.953e-03;
            -8.953e-03,  2.268e-02, -8.945e-03;
            -8.953e-03, -8.945e-03,  2.268e-02];

I_w_full = [1.914e-04, -2.200e-11, -4.000e-12;
            -2.200e-11,  1.914e-04,  2.000e-12;
            -4.000e-12,  2.000e-12,  3.822e-04];

[~, I_w_diagonal] = eig(I_w_full);
I_w = max(diag(I_w_diagonal)); 

I_c = I_cubli(1,1);
I_bar_c = I_c - I_w;

%% Symbolic Dynamics (1D Non-Linear ODE)

syms theta omega_c omega_w u
x_sym = [theta; omega_c; omega_w];

tau_g = m * g * l * sin(theta);

omega_dot_c = (tau_g - u) / I_bar_c;
omega_dot_w = (u / I_w) - omega_dot_c;
dtheta = omega_c;

x_dot_sym = [dtheta; omega_dot_c; omega_dot_w];

dyn_func = matlabFunction(x_dot_sym, 'Vars', {x_sym, u});

%% Fall Simulation Setup

x0 = [deg2rad(theta0_deg); 0; 0];
u_free_fall = 0;

theta_stop_rad = deg2rad(theta_end_deg); 
tspan = [0 10];

opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'Events', @(t,x) fall_event_1d(t, x, theta_stop_rad));
[t_sim, x_sim] = ode45(@(t,x) dyn_func(x, u_free_fall), tspan, x0, opts);

%% Kinematics & Required Effort

theta_sim = x_sim(:, 1);
omega_cubli_sim = x_sim(:, 2);


tau_g_sim = m * g * l * sin(theta_sim);

L_cubli = cumtrapz(t_sim, tau_g_sim); 
omega_w_req_cubli = L_cubli ./ I_w; 
rpm_req_cubli_sim = omega_w_req_cubli * (60 / (2*pi));

t_fall = t_sim(end) - t_sim(1);
omega_dot_w_req_cubli = max(abs(omega_w_req_cubli)) / t_fall;
tau_dynamic = I_w * omega_dot_w_req_cubli;

max_omega_cubli = max(abs(omega_cubli_sim)) * (180/pi); 
max_tau_req = max(abs(tau_g_sim));
total_tau_req = max_tau_req + tau_dynamic;
max_rpm_req = max(abs(rpm_req_cubli_sim));

%% Output Report

fprintf('\n==========================================================\n');
fprintf('%-45s %.1f -> %.1f deg\n', 'Stability Window:', theta0_deg, theta_end_deg);
fprintf('----------------------------------------------------------\n');
fprintf('MINIMUM REQUIRED ANGULAR VELOCITY [deg/s]\n');
fprintf('  %.4f\n', max_omega_cubli);
fprintf('SF REQUIRED ANGULAR VELOCITY [deg/s]\n');
fprintf('  %.4f \n', max_omega_cubli * safety_factor);
fprintf('----------------------------------------------------------\n');
fprintf('MINIMUM REQUIRED MOTOR TORQUE [N*m]\n');
fprintf('  %.4f\n', total_tau_req);
fprintf('SF REQUIRED MOTOR TORQUE [N*m] \n');
fprintf('   %.4f\n', total_tau_req * safety_factor);
fprintf('--------------------------------------------------\n');
fprintf('MINIMUM REQUIRED REACTION WHEEL RPM [RPM]\n');
fprintf('  %.1f\n', max_rpm_req);
fprintf('SF REQUIRED REACTION WHEEL RPM [RPM]\n');
fprintf('  %.1f \n', max_rpm_req * safety_factor);
fprintf('==========================================================\n\n');

%% Plotting

figure('Name', '1D Cubli Initial Fall Simulation', 'Position', [100, 100, 600, 400]);
theta_sim_deg = rad2deg(theta_sim);

plot(t_sim, theta_sim_deg, 'b', 'LineWidth', 2);
grid on;
title('1D Edge Tilt Error During Fall');
xlabel('Time (s)');
ylabel('Angle (deg)');
xlim([0, t_sim(end)*1.1]);

%% Helper Function

function [value, isterminal, direction] = fall_event_1d(~, x, theta_stop_rad)
    current_tilt = abs(x(1));
    
    value = current_tilt - theta_stop_rad;
    isterminal = 1;
    direction = 1;
end