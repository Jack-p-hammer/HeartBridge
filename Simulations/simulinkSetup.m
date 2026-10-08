close all;
clc;
clearvars;

%% Motor Specs

J_m = 2.367e-6 % Motor Intertia
K_v = 276.994 % Rpm / V
K_torque = (3/2)*(1/sqrt(3))*(60/(2*pi))/K_v % Nm/A

b = 286.8; % Chest damping
k = 10490; % Spring constant of chest

%% Motor Controller Setup

iRateLimit = -1;
iLimit = 10;

servo_max_current = 120; % A
servo_pwm_rate_hz = 15000;
Ts = 1/servo_pwm_rate_hz;

pid_dq_hz = 100;

tau_d = 1.5*Ts; % PWM Delay, s

accel_limit = -1;      % rad/s^2 (moteus default)
vel_limit = -1;      % rad/s, example

%% Moteus PLL Filter

zeta_pll = 1;
f_pll = 400; % hz
w_n_pll = 2*pi*f_pll/2.48; % rad/s

%% System Setup
F_max = 500; % N
R_phase = 0; % Ohms, motor winding resistance placeholder
V_batt = 18; % V, minimum for conservative estimate

% Ball screw
D_ball = 0.25*0.0254; % Ball screw diameter
length_ball = 0.2344166;
rho_ball = 7850; % Ball screw density, kg/m^3
eta_ball = 0.8; % Ball Screw efficiency
L_ball = 12.7*1e-3; % m, Ball screw lead
p_ball = L_ball/(2*pi); % Ball screw transmission ratio
J_ball = (pi*rho_ball*length_ball*D_ball^4)/32;
m_ball = 2; % Nut + end effector, kg
J_eff_ball = J_m + J_ball + m_ball*p_ball^2/eta_ball;


tau_max_ball = F_max*L_ball/(2*pi*eta_ball); % Nm

% Rack & Pinion
R = 0.01; % Pinion gear effective radius
J = J_m + 1.72e-7; % Motor and pinion moment of inertia
m = 2*.343; % Mass of rack and plunger