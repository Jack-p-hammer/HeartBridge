close all;
clc;
clearvars;

%% Motor Specs

j_m = 2.367e-6 % Motor Intertia
K_v = 276.994 % Rpm / V
K_torque = (3/2)*(1/sqrt(3))*(60/(2*pi))/K_v % Nm/A

R = 0.01; % Pinion gear effective radius
J = j_m + 1.72e-7; % Motor and pinion moment of inertia
m = 2*.343; % Mass of rack and plunger
b = 0; % Rotational damping coefficient (includes back emf)
c = 286.8; % Linear damping coefficient (includes friction and damping from chest compressions)
k = .9*10490; % Spring constant of chest

%% Motor Controller Setup

iRateLimit = -1;
iLimit = inf;
tau_max = 5; % Nm

servo_max_current = 120; % A
servo_pwm_rate_hz = 15000;
Ts = 1/servo_pwm_rate_hz;

pid_dq_hz = 100;

tau_d = 1.5*Ts; % PWM Delay, s

accel_limit = 50*2*pi;      % rad/s^2 (moteus default)
vel_limit = 20*2*pi;      % rad/s, example

%% Moteus PLL Filter

zeta_pll = 1;
f_pll = 400; % hz
w_n_pll = 2*pi*f_pll/2.48; % rad/s

%% System Setup

N = 5; % Gearbox ratio
nu = 0.95; % Gearbox efficiency
