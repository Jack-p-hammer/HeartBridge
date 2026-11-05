close all;
clc;
clearvars;

%% Motor Specs

J_m = 2.367e-6; % Motor Intertia
K_v = 276.994; % Rpm / V
K_torque = (3/2)*(1/sqrt(3))*(60/(2*pi))/K_v; % Nm/A

b = 286.8; % Chest damping
k = 10490; % Spring constant of chest

%% Moteus PLL Filter

zeta_pll = 1;
f_pll = 200; % Hz, 3 dB cutoff (motor_position.sources.X.pll_filter_hz), calibration overwrites this, set to default
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
pid_kp = 6;   % Nm/rev       (was 1.5 Nm/rad)
pid_kd = 0.05; % Nm/(rev/s)   (was 0.0144 Nm/(rad/s))
pid_ki = 0;        % Nm/s per rev
iRateLimit = -1;   % Nm/s, <0 = no limit (servo.pid_position.iratelimit)

servo_max_current = 40; % A
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
tau_max_ball = min((F_max + m_ball*g)*p_ball/eta_ball, 1.7/1.5); % Nm, per-command maximum_torque (not a config)
iLimit = 1.2*tau_max_ball; % Nm (servo.pid_position.ilimit)