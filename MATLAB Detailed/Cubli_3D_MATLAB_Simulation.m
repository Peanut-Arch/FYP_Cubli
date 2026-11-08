clear; clc; close all;

%% System Parameters

g = 9.81;
m_body = 1.345008;

r_body_com = [0.063282; 0.063233; 0.063249];

I_body = [ 0.014970, -0.005217, -0.005220;
          -0.005217,  0.014980, -0.005218;
          -0.005220, -0.005218,  0.014970]; 

I_rw_full = [ 1.915e-04,  4.200e-11, -3.000e-12;
          4.200e-11,  1.915e-04, -2.000e-12;
         -3.000e-12, -2.000e-12,  3.823e-04];

[~, I_rw_diagonal] = eig(I_rw_full);
spinning_axis = max(diag(I_rw_diagonal));
I_w     = spinning_axis * eye(3);
I_bar_c = I_body - I_w;

%% Equilibrium State Definition (corner-balance point)

com_dir   = r_body_com / norm(r_body_com);

global_up = [0; 0; 1];

rot_axis  = cross(com_dir, global_up);
rot_axis  = rot_axis / norm(rot_axis);
rot_angle = acos(dot(com_dir, global_up));

q0 = [cos(rot_angle/2); sin(rot_angle/2) * rot_axis];
if isrow(q0), q0 = q0'; end

w_c0 = [0; 0; 0];
w_w0 = [0; 0; 0];
x0 = [q0; w_c0; w_w0];
u0 = [0; 0; 0];

%% Symbolic Dynamics (3D Non-Linear ODE)

syms q1 q2 q3 q4 w_c1 w_c2 w_c3 w_w1 w_w2 w_w3
syms u1 u2 u3
q   = [q1; q2; q3; q4];
w_c = [w_c1; w_c2; w_c3];
w_w = [w_w1; w_w2; w_w3];
x_sym = [q; w_c; w_w];
u_sym = [u1; u2; u3];

R_B2G = [ 1-2*(x_sym(3)^2+x_sym(4)^2),      2*(x_sym(2)*x_sym(3) - x_sym(1)*x_sym(4)), 2*(x_sym(2)*x_sym(4) + x_sym(1)*x_sym(3));
          2*(x_sym(2)*x_sym(3) + x_sym(1)*x_sym(4)), 1-2*(x_sym(2)^2+x_sym(4)^2),      2*(x_sym(3)*x_sym(4) - x_sym(1)*x_sym(2));
          2*(x_sym(2)*x_sym(4) - x_sym(1)*x_sym(3)), 2*(x_sym(3)*x_sym(4) + x_sym(1)*x_sym(2)), 1-2*(x_sym(2)^2+x_sym(3)^2) ];

g_body = R_B2G' * [0; 0; -g];
torque_gravity = cross(r_body_com, m_body * g_body);

H_total = (I_bar_c * w_c) + (I_w * w_w);
rhs_c   = -cross(w_c, H_total) + torque_gravity - u_sym;
alpha_c = I_bar_c \ rhs_c;

rhs_w   = u_sym;
alpha_w = (I_w \ rhs_w) - alpha_c;

Omega = [ 0,     -x_sym(5), -x_sym(6), -x_sym(7);
          x_sym(5), 0,       x_sym(7), -x_sym(6);
          x_sym(6), -x_sym(7), 0,       x_sym(5);
          x_sym(7),  x_sym(6), -x_sym(5), 0];
dqdt = 0.5 * Omega * q;

dxdt_sym = [dqdt; alpha_c; alpha_w];
%% Linearization and LQR Design

A_sym = jacobian(dxdt_sym, x_sym);
B_sym = jacobian(dxdt_sym, u_sym);

A = double(subs(A_sym, [x_sym; u_sym], [x0; u0]));
B = double(subs(B_sym, [x_sym; u_sym], [x0; u0]));

E_q = [ -q0(2), -q0(3), -q0(4);
         q0(1), -q0(4),  q0(3);
         q0(4),  q0(1), -q0(2);
        -q0(3),  q0(2),  q0(1) ];

T_quat = blkdiag(E_q, eye(3), eye(3));
A_red  = T_quat' * A * T_quat;
B_red  = T_quat' * B;

[Abar, Bbar, ~, T_ctrb, k_ctrb] = ctrbf(A_red, B_red, eye(9));
n_controllable   = sum(k_ctrb);
n_uncontrollable = 9 - n_controllable;

Ac = Abar(n_uncontrollable+1:end, n_uncontrollable+1:end);
Bc = Bbar(n_uncontrollable+1:end, :);

% LQR Tuning Weights
q_orientation = 5e5;    % Penalty on 3D attitude error
q_body_speed  = 1e1;    % Penalty on body rotation rate
q_wheel_speed = 1e-4;   % Penalty on reaction wheel speed

r_torque_use  = 5e3;    % Penalty on motor torque output

Q_red = diag([q_orientation, q_orientation, q_orientation, ...
              q_body_speed,  q_body_speed,  q_body_speed, ...
              q_wheel_speed, q_wheel_speed, q_wheel_speed]);
R = diag([r_torque_use, r_torque_use, r_torque_use]);

Qbar = T_ctrb * Q_red * T_ctrb';
Qc   = Qbar(n_uncontrollable+1:end, n_uncontrollable+1:end);

Kc = lqr(Ac, Bc, Qc, R);

Kbar  = [zeros(3, n_uncontrollable), Kc];
K_red = Kbar * T_ctrb;
K     = K_red * T_quat';


use_custom = false;  kTh = -7.1;  kOm = -0.45;  kW = -0.002;
if use_custom
    u  = ones(3,1)/sqrt(3);  P = eye(3) - u*u';
    Kp = [kTh*P, kOm*P, kW*P];
    Kz = Kp * blkdiag(2*eye(3), eye(3), eye(3));
    K  = [Kz(:,1:3)*E_q', Kz(:,4:9)];
end


% Matrix to map Body Axes [X; Y; Z] -> Physical Motors [M1(Y); M2(X); M3(Z)]
P_motors = [0 1 0;   % Motor 1 = Y axis
            1 0 0;   % Motor 2 = X axis
            0 0 1];  % Motor 3 = Z axis

K_embedded = P_motors * K;
K_embedded(:, [8, 9]) = K_embedded(:, [9, 8]);

for i = 1:size(K_embedded, 1)
    row   = K_embedded(i, :);
    elems = arrayfun(@(v) sprintf('%14.12gf', v), row, 'UniformOutput', false);
    fprintf('  { %s },\n', strjoin(elems, ', '));
end

save('K_3D.mat', 'K_embedded');

d1GainsFromK(K_embedded, x0);

report_control_bandwidth(Ac, Bc, Kc);
%% Closed-Loop Simulation Setup

dyn_func = matlabFunction(dxdt_sym, 'Vars', {x_sym, u_sym});

sim_degrees = 2;
tilt_angle  = deg2rad(sim_degrees);
q_tilt      = [cos(tilt_angle/2), sin(tilt_angle/2), 0, 0];
q_start     = quatmultiply(q_tilt, q0')';
x_start     = [q_start; 0; 0; 0; 0; 0; 0];
tspan       = [0, 0.5];

closed_loop_ode = @(t, x_curr) apply_foc_saturation(x_curr, x0, K, dyn_func);
%% Run Simulation & Plot Results


[t_out, x_out] = ode45(closed_loop_ode, tspan, x_start);

u_requested = zeros(length(t_out), 3);
u_applied   = zeros(length(t_out), 3);
tilt_true = zeros(length(t_out), 1);
yaw_deg   = zeros(length(t_out), 1);
q0c = [q0(1); -q0(2); -q0(3); -q0(4)];

for i = 1:length(t_out)
    [~, u_req_i, u_app_i] = apply_foc_saturation(x_out(i, :)', x0, K, dyn_func);
    u_requested(i, :) = u_req_i';
    u_applied(i, :)   = u_app_i';

    qc = x_out(i, 1:4)';
    
    R_mat = [1-2*(qc(3)^2+qc(4)^2),   2*(qc(2)*qc(3)-qc(1)*qc(4)), 2*(qc(2)*qc(4)+qc(1)*qc(3));
             2*(qc(2)*qc(3)+qc(1)*qc(4)), 1-2*(qc(2)^2+qc(4)^2),   2*(qc(3)*qc(4)-qc(1)*qc(2));
             2*(qc(2)*qc(4)-qc(1)*qc(3)), 2*(qc(3)*qc(4)+qc(1)*qc(2)), 1-2*(qc(2)^2+qc(3)^2)];
    up_b = R_mat * com_dir;
    tilt_true(i) = rad2deg(acos(min(1, max(-1, up_b(3)))));
    
    qe = quatmultiply(x_out(i, 1:4), q0c'); 
    if qe(1) < 0, qe = -qe; end
    yaw_deg(i) = rad2deg(2*qe(4));
end
disp(max(abs(u_requested(:))));
figure('Name', '3D Cubli LQR Corner Stabilization', 'Position', [100, 100, 900, 700]);

subplot(3, 1, 1);
plot(t_out, tilt_true, 'b', 'LineWidth', 2); hold on;
plot(t_out, yaw_deg, 'r--', 'LineWidth', 1.5);
grid on; title('3D Corner Attitude Error'); ylabel('Angle (deg)');
legend('True Tilt (Fall)', 'Yaw (Twist)', 'Location', 'eastoutside');

subplot(3, 1, 2);
plot(t_out, u_requested, '--', 'LineWidth', 1.5); hold on;
set(gca, 'ColorOrderIndex', 1);
plot(t_out, u_applied, '-', 'LineWidth', 2);
yline(0.35, ':k', 'Rated Limit (0.35 Nm)'); yline(-0.35, ':k');
grid on; title('Motor Torque: Requested (--) vs Saturated (-)'); ylabel('Torque (Nm)');
legend('X-Axis req', 'Y-Axis req', 'Z-Axis req', 'X-Axis applied', 'Y-Axis applied', ...
    'Z-Axis applied', 'Location', 'eastoutside');

subplot(3, 1, 3);
plot(t_out, x_out(:, 8:10) * (60/(2*pi)), 'LineWidth', 2);
grid on; title('Reaction Wheel Speeds'); ylabel('Speed (RPM)'); xlabel('Time (s)');
legend('X-Axis Wheel', 'Y-Axis Wheel', 'Z-Axis Wheel', 'Location', 'eastoutside');
%% LOCAL FUNCTION DEFINITIONS

function [dx, u_req, u_applied] = apply_foc_saturation(x_curr, x0, K, dyn_func)

    u_req = -1 * (K * (x_curr - x0));

    omega_w = x_curr(8:10);
    R_MOTOR   = 4.3;
    KE = [0.157; 0.157; 0.157];
    KT = [0.25; 025; 0.25];
    TAU_RATED = 0.35;
    V_MAX     = 12;
    I_LIM     = 1.8;

    u_applied = zeros(3, 1);
    v_applied = zeros(3, 1);

    for j = 1:3
        Ihi = min( I_LIM, ( V_MAX - KE(j)*omega_w(j)) / R_MOTOR);
        Ilo = max(-I_LIM, (-V_MAX - KE(j)*omega_w(j)) / R_MOTOR);
        
        tHi = min( TAU_RATED, KT(j)*Ihi);
        tLo = max(-TAU_RATED, KT(j)*Ilo);
        
        tau = max(tLo, min(tHi, u_req(j)));
        
        u_applied(j) = tau;
        v_applied(j) = (tau / KT(j)) * R_MOTOR + KE(j)*omega_w(j);
    end

    dx = dyn_func(x_curr, u_applied);
end

function report_control_bandwidth(A, B, K)

    Acl   = A - B*K;
    poles = eig(Acl);
    wn    = abs(poles);
    f_hz  = wn / (2*pi);

    [f_max, ~] = max(f_hz);

    fprintf('------------------------------------\n');
    fprintf('%.1f - %.1f Hz  (dt: %.1f - %.1f us)\n', ...
            10*f_max, 20*f_max, ...
            1e6/(20*f_max), 1e6/(10*f_max));
    fprintf('------------------------------------\n');
end

function [kTheta, kOmega, kW] = d1GainsFromK(K, x0)

    if nargin < 2, x0 = [0.8881 0.325 -0.3252 0]; end
    x0 = x0(:).' / norm(x0);

    motorAxis = [2 1 3];

    Hq = zeros(3);
    for a = 1:3
        ea = zeros(1,4);  ea(a+1) = 1;
        d  = 0.5 * qmul(x0, ea);
        Hq(:,a) = K(:,1:4) * d.';
    end

    axisNames = 'XYZ';
    kTheta = zeros(1,3); kOmega = zeros(1,3); kW = zeros(1,3);
    for m = 1:3
        ax        = motorAxis(m);
        kTheta(m) = Hq(m, ax);
        kOmega(m) = K(m, 4 + ax);
        kW(m)     = K(m, 7 + m);

        oa = setdiff(1:3, ax);
        om = setdiff(1:3, m);
        kThetaE(m) = Hq(m,ax) - mean(Hq(m,oa));
        kOmegaE(m) = K(m,4+ax) - mean(K(m,4+oa));
        kWE(m)     = K(m,7+m)  - mean(K(m,7+om));

        fprintf('1D    :  M%d (axis %s): kTheta = %9.4f   kOmega = %8.4f   kW = %8.5f\n', ...
        m, axisNames(ax), kTheta(m), kOmega(m), kW(m));

        fprintf('3D Eq :  M%d (axis %s): kTheta = %9.4f   kOmega = %8.4f   kW = %8.5f\n', ...
        m, axisNames(ax), kThetaE(m), kOmegaE(m), kWE(m));
    end
end

function q = qmul(a, b)
    q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
         a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3), ...
         a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2), ...
         a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end