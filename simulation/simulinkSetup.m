close all;
clc;
clearvars;

%% Motor Specs

J_m = 2.367e-6; % Motor Intertia
K_v = 276.994; % Rpm / V
K_torque = (3/2)*(1/sqrt(3))*(60/(2*pi))/K_v; % Nm/A

%% Chest Model (adult, start of CPR)
% Linear fit F = k*x + d*xdot to 615 out-of-hospital patients, with separate
% compression / recoil damping (Ruiz de Gauna et al. 2023, Comput Methods
% Programs Biomed 242:107847)
k = 10490;   % N/m, chest stiffness
b = 286.8;   % N*s/m, compression damping
b_r = 488.9; % N*s/m, recoil damping

% Nonlinear chest: stiffening spring with the shape of Tsitlik et al. 1983
% (F = 54.9 N/cm*x + 10.8 N/cm^2*x^2) and damping that grows with depth,
% d(x) = slope*(x_d0 + x) (Gruben et al. 1993/1999, Neurauter et al. 2009).
% Each is scaled so its linear fit over the stroke equals k, b, b_r above.
chest_nonlinear = true; % false = linear k with b in both phases (previous model)
chest_stroke = 0.05;    % m, depth range the scaling is fitted over
k1_tsitlik = 5490;      % N/m
k2_tsitlik = 1.08e5;    % N/m^2
x_d0 = 0.01;            % m, damping at zero depth as a depth offset (Gruben human: ~75 N*s/m at x = 0)
k_lin_tsitlik = k1_tsitlik + 3/4*k2_tsitlik*chest_stroke; % least-squares linear fit over the stroke
chest_p = struct( ...
    'nonlinear', double(chest_nonlinear), 'k', k, 'b', b, ...
    'k1', k1_tsitlik*k/k_lin_tsitlik, 'k2', k2_tsitlik*k/k_lin_tsitlik, 'x_d0', x_d0, ...
    'dc_slope', b/(x_d0 + chest_stroke/2), 'dr_slope', b_r/(x_d0 + chest_stroke/2));

%% Moteus PLL Filter

zeta_pll = 1;
f_pll = 400; % Hz, 3 dB cutoff (motor_position.sources.X.pll_filter_hz), calibration overwrites this, reset it after
w_n_pll = 2*pi*f_pll/2.48; % rad/s, filter natural frequency (not an angle)

%% System Setup
F_max = 750; % N
g = -9.81; % m/s^2, gravity along x (vertical axis, x < 0 is down into the chest)
R_phase = 0.047; % Ohms, motor winding resistance
V_batt = 22.2; % V, minimum for conservative estimate

% Ball screw
D_ball = 0.25*0.0254; % Ball screw diameter
length_ball = 0.2344166;
rho_ball = 7850; % Ball screw density, kg/m^3
eta_ball = 0.8; % Ball Screw efficiency
L_ball = 12.7*1e-3; % m/rev, Ball screw lead
p_ball = L_ball/(2*pi); % m/rad, Ball screw transmission ratio
J_ball = (pi*rho_ball*length_ball*D_ball^4)/32;
m_ball = 2; % Nut + end effector, kg
J_eff_ball = J_m + J_ball + m_ball*p_ball^2/eta_ball;

% tau_max_ball = inf;

% Rack & Pinion
R = 0.01; % Pinion gear effective radius
J = J_m + 1.72e-7; % Motor and pinion moment of inertia
m = 2*.343; % Mass of rack and plunger

%% Motor Controller Setup
% Controller-side quantities use moteus units (positions in rev at the
% output, rotor_to_output_ratio = 1). The plant stays in SI (rad, m).

% servo.pid_position
pid_kp = 4;   % Nm/rev       (was 1.5 Nm/rad)
pid_kd = 0.02; % Nm/(rev/s)   (was 0.0144 Nm/(rad/s))
pid_ki = 0;        % Nm/s per rev
iRateLimit = -1;   % Nm/s, <0 = no limit (servo.pid_position.iratelimit)

servo_max_current = 45.5; % A
servo_pwm_rate_hz = 15000;
Ts = 1/servo_pwm_rate_hz;

pid_dq_hz = 200; % Hz (servo.pid_dq_hz), calibration overwrites this, so set to default

tau_d = 1.5*Ts; % PWM Delay, s

% Trajectory limits are the config defaults, which apply because the Pi
% sends velocity_limit/accel_limit = NaN. Config NaN = no limit; don't use
% negative here (only a negative per-command limit disables it).
accel_limit = 16.5/L_ball;   % rev/s^2 (servo.default_accel_limit, firmware default 50)
vel_limit = 0.6/L_ball; % rev/s (0.6 m/s at the nut) (servo.default_velocity_limit)
inertia_ff = J_eff_ball; % kg*m^2 (servo.inertia_feedforward), default 0 = off
ff_torque = 0; % Nm, per-command feedforward_torque, holds the end effector's weight
tau_max_ball = min((F_max + m_ball*g)*p_ball/eta_ball, 1.7/1.25); % Nm, per-command maximum_torque (not a config)
iLimit = 1.2*tau_max_ball; % Nm (servo.pid_position.ilimit)