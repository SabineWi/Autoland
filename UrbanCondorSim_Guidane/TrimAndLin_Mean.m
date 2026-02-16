%% ================ Trim and Lin Function - Description ===================
% Name: TrimAndLin
% Institution: TU Dresden - Chair of Flight Mechanics and Flight Control
% Author: Ole Ostermann
%
% Description:
% Script for initialising the Simulink simulation, trimming an aicraft
% and linearisation. -> Starting point for each simulation!
%
% Simulation:
% The simulation is a 6-DOF airplane model on flat earth coordinates
% but with parallel WGS-84 coordinate calculation. It has an ISA model of
% the atmosphere, a 1-cos and step wind gust model and joystick input as
% well as a Flight Gear interface for visualisation.
%
% Options:
% In this script an implemented aircraft can be chosen and configured,
% as well as several options for the flight condition to be trimmed and
% the desired LTI models. Wind settings can be adjusted seperately in
% the "Wind_Settings" script.
%
% Current Limitations:
%   - no time varying mass
%   - no time varying c.g.
%   - no assymetries allowed
%       --> one basic configuration of the aircraft
%   - no wind turbulences
%   - no ILS
%   - no stall model
clear;clc


%% ----------------------- Simulation Settings ----------------------------

Settings.Simulation.t_start = 0;
Settings.Simulation.t_end   = 30; % not important for the trim at all
Settings.Simulation.step_size = 0.0125; % = 80 Hz (ONERA: 0.05)


%% -------------------------- Initialisations -----------------------------

% Include paths
InitSimulationPaths

% Sets the initial values for the integrators
InitTrimValues

% Load model so that Simulink-model paths can be found
load_system('Flight_Simulation');

% comment out FlightGear & Joystick interface for normal simulations
set_param('Flight_Simulation/FlightGear_Interface/FG_simpleInterface','commented','on');
set_param('Flight_Simulation/Joystick_Input','commented','on');

% Prepare FlightGear interface & Joystick interface
% InitFlightGearInterface


%% ------------------- Select and load aircraft data ----------------------
% Load aircraft constants and aerodynamics by calling InitAircraft_XX
% from one of the implemented aircrafts

InitAircraft_UrbanCondor_Mean

%% --------------- Set the Trim Point and Flight Condition ----------------

% Set trim altitude
%   -> if Trim.States.Alt is an array then a multitrim is executed
Trim.States.Alt = [100]; %[100]

% Set trim speed velocity
%   -> if Trim.States.Vel is an array then a multitrim is executed
Trim.States.Vel = [20]; %[17] [m/s] 
% (For ONERA comparison: H=300, V=65.934245, gamma=-3 & change atmosphere & dt)

% Set trim path angle gamma for ascending or descending flight
Trim.States.Gam = 0; % [deg]

% Set trim turn rate for coordinated turn
Trim.States.Psidot = 0; % [deg/s]

% Set trim loadfactor delta_n_z for symmetrical pull-up or push-over
Trim.States.Nz_delta = 0; % [g] -> Nz_delta = -0.1 g becomes nz = 0.9 g

% Set trim position in WGS-84 coordinates
Trim.States.mue     = 51.133080; % [deg];
Trim.States.lambda  = 13.768361; % [deg];
Trim.States.h       = Trim.States.Alt;
% sets a starting position for visualisation (Dresden Airport EDDC)


%% ------------------- Aircraft Configuration -----------------------------

% Set landing gear on & off with 1 & 0 respectively
Aircraft.LandingGearFlg     = 0;
% Remark: If AC doesn't support landing gear, this won't change anything

% Set flaps (check step widths for your aircraft)
Aircraft.Flaps_degrees      = 0; % [deg]
% Remark: If AC doesn't support flaps, this won't change anything
% Steps widths:
% P44: 0,10,25,40 deg
% A320: 0,15,20,40 deg

% Set speedbrakes percentage
Aircraft.Speedbrakes        = 0; % [%]
% Remark: If AC doesn't support speedbrakes, this won't change anything

% Set autopilot gains
SetAutopilot % autopilot is bypassed during trim and linearisation
% currently only altitude hold control


%% ----------------------- Environment Settings ---------------------------
% Remark:
% Simulink wind module (time dependant) is bypassed during TrimAndLin.
% However, a constant wind velocity input remains active to trim and
% linearise in constant wind velocities. It is set to 0 by default.
Wind_Settings
% Settings in a seperate script to allow changes without rerunning Trim

% Set environment constants
InitEnvironmentConstants

% Set compressibility effects on or off (for dynamic pressure calculation)
Settings.Compressibility = 0; % 1 = on, 0 = off


%% -------------------------- Lin Settings --------------------------------
% Choose desired linearisations, multiple possible

% ---------------- Longitudinal Linearisation Settings --------------------
%
% StateSpaceLong:   'theta_4x4'     -> 4x4 longitudinal with theta as state
%                   'ShortPeriod'   -> 2x2 states alpha and q representing the short time dynamics
%                   'Phugoid'       -> 2x2 simple long time dynamics (simple phugoid approximation)
%                   'n_x_z'         -> 4x4 adding n_z & n_x outputs
%                   'speedbrakes'   -> 4x4 with speedbrake input

Settings.StateSpaceLong     = {'theta_4x4','n_x_z','ShortPeriod','Phugoid','speedbrakes'};

% ------------------- Lateral Linearisation Settings ----------------------
%
% StateSpaceLat:    '4x4'           -> 4x4 lateral with p, r, beta, phi
%                   'RollMode'      -> 2x2 simple roll mode approximation
%                   'DutchRoll'     -> 2x2 simple dutch roll mode approximation
%                   'n_y'           -> 4x4 adding n_y output

Settings.StateSpaceLat     = {'n_y','4x4','RollMode','DutchRoll'};


%% ------------------------------- Trim -----------------------------------
% Comment out wind models & autopilot models during Trim and Lin
set_param('Flight_Simulation/Environment_Subsystems/Wind_Model','commented','on');
set_param('Flight_Simulation/Controller/Autopilot','commented','on');

% Run trim function
[Trim, state_idx] = fct_multi_trim(Trim, Aircraft, Environment);
% Info: works with multiple trim points


%% --------------------------- Linearisation ------------------------------

% Run linearisation function
LTI_full = fct_multi_lin(Trim, Environment, Aircraft, state_idx);
% Info: works with multiple trim points


% Uncomment wind models & autopilot after Trim and Lin
set_param('Flight_Simulation/Environment_Subsystems/Wind_Model','commented','off');
set_param('Flight_Simulation/Controller/Autopilot','commented','off');


%% --------------------- Get State Space Models ---------------------------
% Info:
% - works for multiple statespace systems and multi-trim
% - use cell output or varargout/ and varargin type as prefered

% ------------------- Longitudinal State Space Models ---------------------
% uses a cell as output
[LTI_long] = fct_get_state_space_long(LTI_full, state_idx, Trim, Settings);

% uses varargout/ and varargin type - have to be manually set!
% [n_x_z] = fct_get_state_space_long_var(LTI_full,state_idx, Trim, Settings.StateSpaceLong{2});

% --------------------- Lateral State Space Models ------------------------
% uses a cell as output
[LTI_lat] = fct_get_state_space_lat(LTI_full, state_idx, Trim, Settings);

% uses varargout/ and varargin type - have to be manually set!
[lat_4x4] = fct_get_state_space_lat_var(LTI_full, state_idx, Trim, Settings.StateSpaceLat{2});


%% ------------------- Prepare Workspace for Simulation -------------------

U_in.time           = [0]; % just one execution
U_in.signals.values = [Trim.Controls_result.Thrust(1),...
                       Trim.Controls_result.Elevator(1),...
                       Trim.Controls_result.Aileron(1),...
                       Trim.Controls_result.Rudder(1),...
                       Aircraft.Speedbrakes,...
                       Environment.Wind_Input_UVW'];
                   
