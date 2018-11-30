
%% SETUP_LAB_HELI_3D
%
% This script sets the model parameters and designs a PID position
% controller using LQR for the Quanser 3-DOF Helicopter plant.
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
clear all;
%
%% ############### USER-DEFINED 3-DOF HELI CONFIGURATION ###############
% Amplifier Gain used for yaw and pitch axes: set VoltPAQ to 3.
K_AMP = 3;
% Amplifier Maximum Output Voltage (V)
VMAX_AMP = 24;
% Digital-to-Analog Maximum Voltage (V): set to 10 for Q4/Q8 cards
VMAX_DAC = 10;
% Initial elvation angle (rad)
elev_0 = -27.5*pi/180;
%
%% ############### USER-DEFINED CONTROLLER/FILTER DESIGN ###############
% Type of Controller: set it to 'LQR_AUTO' or 'MANUAL'  
CONTROLLER_TYPE = 'LQIF';    % controller design: manual mode
% Is Active Disturbance Mass used: 'YES' or 'NO'
% note: larger gain controller calculated for ADS system
WITH_ADS = 'NO';
% Specifications of a second-order low-pass filter
wcf = 2 * pi * 20; % filter cutting frequency
zetaf = 0.9;        % filter damping ratio
% Anti-windup: integrator saturation (V)
SAT_INT_ERR_ELEV = 7.5;
SAT_INT_ERR_TRAVEL = 7.5;
%
%% ############### USER-DEFINED DESIRED COMMAND SETTINGS ###############
% Note: These limits are imposed on both the program and joystick commands.
% Elevation position command limit (deg)
CMD_ELEV_POS_LIMIT_LOWER = elev_0*180/pi;
CMD_ELEV_POS_LIMIT_UPPER = -CMD_ELEV_POS_LIMIT_LOWER;
% Maximum Rate of Desired Position (rad/s)
CMD_RATE_LIMIT = 45.0 * pi / 180;
%
%% ############### USER-DEFINED JOYSTICK SETTINGS ###############
% Joystick input X sensitivity used for travel (deg/s/V)
K_JOYSTICK_X = 40.0;
% Joystick input Y sensitivity used for elevation (deg/s/V)
K_JOYSTICK_Y = 45.0;
%

%% ############### ACTIVE DISTURBANCE SYSTEM ###############
% Amplifier Gain for ADS: set VoltPAQ to 3.
K_AMP_X = 3;
% Maximum Output Voltage (V)
VMAX_AMP_X = 15;
% Load ADS parameters
[ K_EC_X, POS_HOME, CAL_VEL, CAL_TIME, wcf_x, kp, X_MAX, V_MAX  ] = setup_ads_configuration( );
%

%% ############### MODELING ###############
% These parameters are used for model representation and controller design.
[ Kf, m_h, m_w, m_f, m_b, Lh, La, Lw, g, K_EC_T, K_EC_P, K_EC_E ] = setup_heli_3d_configuration();
