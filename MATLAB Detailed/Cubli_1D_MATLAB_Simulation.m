clear; clc; close all;

%% System Parameters

g = 9.81;

m = 1.362693;
r_com = [0.063169; 0.063119; 0.063135];
l = sqrt(r_com(2)^2 + r_com(3)^2);

I_cubli =    [1.513e-02, -5.262e-03, -5.265e-03;
            -5.262e-03, 1.514e-02, -5.263e-03;
            -5.265e-03, -5.263e-03, 1.514e-02];

I_w_full = [1.986e-04, 4.300e-11, -3.000e-12;
            4.300e-11, 1.986e-04, -2.000e-12;
            -3.000e-12, -2.000e-12, 3.965e-04];

[~, I_w_diagonal] = eig(I_w_full);
I_w = max(diag(I_w_diagonal));

I_c = I_cubli(1,1);
I_bar_c = I_c - I_w;

%% Symbolic Dynamics

x0 = [0; 0; 0];
u0 = 0;

syms theta omega_c omega_w u
x_sym = [theta; omega_c; omega_w];

tau_g = m * g * l * sin(theta);

omega_dot_c = (tau_g - u) / I_bar_c;
omega_dot_w = (u / I_w) - omega_dot_c;
dtheta = omega_c;

dxdt_sym = [dtheta; omega_dot_c; omega_dot_w];

%% Linearisation and LQR Design

A_sym = jacobian(dxdt_sym, x_sym);
B_sym = jacobian(dxdt_sym, u);

A = double(subs(A_sym, [x_sym; u], [x0; u0]));
B = double(subs(B_sym, [x_sym; u], [x0; u0]));

% LQR Weights

q_theta = 1.4e5;    % Penalty on 3D attitude error
q_omega_c  = 2e2;    % Penalty on body rotation rate
q_omega_w = 1e-2;   % Penalty on reaction wheel speed

r_torque  = 5e3;    % Penalty on motor torque output


Q = diag([q_theta, q_omega_c, q_omega_w]);
R = r_torque;

K = lqr(A, B, Q, R);

disp('LQR Gain K:');
fprintf('{ %3.5ff, %3.5ff, %3.5ff };\n', K(1), K(2), K(3));

report_control_bandwidth(A, B, K);

%% Closed-Loop Simulation Setup

dyn_func = matlabFunction(dxdt_sym, 'Vars', {x_sym, u});

sim_degrees = 2; % Start Tilt

x_start = [deg2rad(sim_degrees); 0; 0];
tspan = [0, 1];

closed_loop_ode = @(t, x_curr) apply_foc_saturation_1d(x_curr, x0, K, dyn_func);
[t_out, x_out] = ode45(closed_loop_ode, tspan, x_start);

%% Plot Results

u_requested = zeros(length(t_out), 1);
u_applied   = zeros(length(t_out), 1);

for i = 1:length(t_out)
    [~, u_requested(i), u_applied(i)] = apply_foc_saturation_1d(x_out(i,:)', x0, K, dyn_func);
end
disp(['requested torque: ' num2str(u_requested(1))]);

figure('Name', '1D Cubli LQR Edge Stabilization', 'Position', [100, 100, 900, 700]);

subplot(3, 1, 1);
plot(t_out, rad2deg(x_out(:,1)), 'b', 'LineWidth', 2);
grid on; title('1D Edge Tilt Error'); ylabel('Angle (deg)');

subplot(3, 1, 2);
plot(t_out, u_requested, '--r', 'LineWidth', 2); hold on;
plot(t_out, u_applied, 'b', 'LineWidth', 1.5);
yline(0.35, ':k', 'Rated Limit (0.35 Nm)'); yline(-0.35, ':k');
grid on; title('Motor Torque: Requested vs Saturated'); ylabel('Torque (Nm)');
legend('Requested', 'Saturated', 'Location', 'northeast');

subplot(3, 1, 3);
plot(t_out, x_out(:,3) * (60/(2*pi)), 'g', 'LineWidth', 2);
grid on; title('Reaction Wheel Speed'); ylabel('Speed (RPM)'); xlabel('Time (s)');
%% Local Function Definition

function [dx, u_req, u_applied] = apply_foc_saturation_1d(x_curr, x0, K, dyn_func)

    u_req = -1 * (K * (x_curr - x0));

    omega_w = x_curr(3); 
    R_motor = 4.3; 
    Ke = 0.157; 
    Kt = 0.25; 
    tau_rated = 0.35;
    V_max = 12; 

    I_max = max(0, (V_max - Ke * abs(omega_w)) / R_motor);
    tau_limit = min(tau_rated, Kt * I_max);
    u_applied = max(-tau_limit, min(tau_limit, u_req));

    dx = dyn_func(x_curr, u_applied);
end

function report_control_bandwidth(A, B, K)

    Acl   = A - B*K;
    poles = eig(Acl);
    wn    = abs(poles);
    f_hz  = wn / (2*pi);
    [f_max, ~] = max(f_hz);

    fprintf('\n--- Closed-loop bandwidth check ---\n');
    fprintf('Recommended control rate: %.1f - %.1f Hz  (dt: %.1f - %.1f us)\n', ...
            10*f_max, 20*f_max, ...
            1e6/(20*f_max), 1e6/(10*f_max));
    fprintf('------------------------------------\n');
end