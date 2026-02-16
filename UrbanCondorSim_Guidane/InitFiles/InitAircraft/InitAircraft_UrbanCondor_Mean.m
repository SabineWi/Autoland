%% ------------------ Load UrbanCondor Aircraft ---------------------------
% Initialize Aircraft constants, aerodynamics and trim parameters

% Representation of the Piper PA-44 aircraft. Data taken from the
% professional ALSIM flight training simulator.
% Flaps Settings are 0,10,25,40
% Attention! Flight profile alleviation factor is only a guess!

Aircraft.Name = 'UrbanCondor';

% Flight envelope limits:
% altitude max: 5200 m
% V_max: 104 m/s
% V_c: 80 m/s
% V_min: 30 m/s (with full flaps)

%% ---------------- Aerodynamics LookUp Table input -----------------------

% loads the look up tables containing the aerodynamic data
%InitAeroCoeff_UrbanCondor_Mean %Mit CMeta aus AVL
InitAeroCoeff_UrbanCondor_Mean_v2 %Mit CMeta aus AVL + Vorzeichen anders

%% -------------- Angle of Attack and Sideslip limits ---------------------

Aircraft.AoA_Limits.max = 15;  % [deg]
Aircraft.AoA_Limits.min = -10; % [deg]

Aircraft.Sideslip_Limits.max = 10;  % [deg]
Aircraft.Sideslip_Limits.min = -10; % [deg]


%% ----------------------- Mass and Dimensions ----------------------------

% Mass properties
Aircraft.Mass_properties.Mass   = 7.672;     %5.000;    %kg
Aircraft.Mass_properties.J_xx   = 0.8553;    %0.239747; %kg*m^2 
Aircraft.Mass_properties.J_yy   = 1.010;     %0.301452; %kg*m^2 
Aircraft.Mass_properties.J_zz   = 1.658;     %0.536532; %kg*m^2 
Aircraft.Mass_properties.J_xz   = 0.1287e-1; %1e-3;     %kg*m^2 (goes in negative) not sure if this is correct (this is the Value felix gave us for the Entwurfprojekt)

% Dimensional properties 
Aircraft.Dimensions.WingSpan    = 2.00;%1.76; %m
Aircraft.Dimensions.WingArea    = 0.66;%0.581; %m^2
Aircraft.Dimensions.MAC         = 0.33; %m

% Center of gravity X-location as a fraction of MAC
Aircraft.Dimensions.cg_ref   = 0.288; % reference point where aerodynamics were measured
Aircraft.Dimensions.cg       = 0.288; % chosen, current c.g.


%% --------------------- Controls properties ------------------------------

Aircraft.Controls.Elev_pos_max  = +20;%+20; %deg
Aircraft.Controls.Elev_pos_min  = -20;%-20; %deg
Aircraft.Controls.Elev_rate_max = +200; %deg/s
Aircraft.Controls.Elev_rate_min = -200; %deg/s
Aircraft.Controls.Elev_lag      = 1/30; %0.02; %s Zeitkonstante des PT1-Glieds = 1/wc

Aircraft.Controls.Ail_pos_max   = +20; %deg
Aircraft.Controls.Ail_pos_min   = -20; %deg
Aircraft.Controls.Ail_rate_max  = +200; %deg/s
Aircraft.Controls.Ail_rate_min  = -200; %deg/s
Aircraft.Controls.Ail_lag       = 1/30; %0.01; %s Zeitkonstante des PT1-Glieds = 1/wc

Aircraft.Controls.Rud_pos_max   = +20; %deg
Aircraft.Controls.Rud_pos_min   = -20; %deg
Aircraft.Controls.Rud_rate_max  = +200; %deg/s
Aircraft.Controls.Rud_rate_min  = -200; %deg/s
Aircraft.Controls.Rud_lag       = 1/30; %0.02; %s Zeitkonstante des PT1-Glieds = 1/wc


%% ---------------------- Engine properties -------------------------------

Aircraft.Engine.Ang_Mom         = 0; %kg*m^2/s
Aircraft.Engine.Prop_Surface    = ((13*0.0254)^2)/4*pi; %m^2 13 inch propeller
Aircraft.Engine.z_eng           = 0; %m

Aircraft.Engine.thrust_max      = 50; %N
Aircraft.Engine.thrust_min      = 0; %N
% should probably be upgraded, seems like dummy values:
Aircraft.Engine.thrust_rate_max = 500; %N/s 
Aircraft.Engine.thrust_rate_min = -500; %N/s
Aircraft.Engine.thrust_lag      = 0.01; %s


%% -------------- Flight Profile Alleviation Factor F_g -------------------
% Flight profile alleviation factor F_g needed for automatic calculation
% of gust amplitudes in compliance with CS-25 standard

% F_g constants
% Set to 0 when unknown!
Aircraft.F_g.Const(1) = 500; % Z_mo: maximum operating altitude
Aircraft.F_g.Const(2) = 5.5;%3.5; % m_TO: maximum take-off weight
Aircraft.F_g.Const(3) = 5.5;%3.5; % m_L: maximum landing weight
Aircraft.F_g.Const(4) = 5.5;%3.5; % m_zF: maximum zero fuel weight - guess

if Aircraft.F_g.Const ~= 0
    % R_1 = m_L / m_TO
    Aircraft.F_g.R_1    = Aircraft.F_g.Const(3) / Aircraft.F_g.Const(2);
    % R_2 = m_zF / m_TO
    Aircraft.F_g.R_2    = Aircraft.F_g.Const(4) / Aircraft.F_g.Const(2);
    % F_gz = 1 - Z_mo/76200
    Aircraft.F_g.F_gz   = 1 - Aircraft.F_g.Const(1)/76200;
    Aircraft.F_g.F_gm   = sqrt(Aircraft.F_g.R_2 * tan(pi*Aircraft.F_g.R_1/4));
    Aircraft.F_g.final  = 0.5 * (Aircraft.F_g.F_gz + Aircraft.F_g.F_gm);
else
    % manually adjust if F_g constants are unknown or F_g is known
    Aircraft.F_g.final  = 0.5; 
    % will be changed to 0.5 by default if set to 0 here
end


%% ---------------- Trim - Optimization initialization --------------------

% ------------------------ Trim Initial Guess -----------------------------
% initial guess for the control inputs used for trim optimisation
Trim.Controls_init.Thrust    = 6;        % [N]
Trim.Controls_init.Elevator  = -3.9;%-2;       % [deg]
Trim.Controls_init.Rudder    = 0;        % [deg]
Trim.Controls_init.Aileron   = 0;        % [deg]

% initial guess for AoA and Sideslip used for trim optimisation
Trim.Controls_init.alpha     = 4.2*(pi/180);%1 *(pi/180);    % AoA [rad]
Trim.Controls_init.beta      = 0 *(pi/180); % Sideslip [rad]
% dummy values used for trim routine size

% ---------------------- weights initialization ---------------------------
% - weights for the costs of the trim optimisation function
% - weight = 0 means that it's a free variable and can be arbitrary without
%   violating the trim condition
% - symmetric configuration is expected
Aircraft.Weight_for_OptCost = [...
    0           ...% 1 npos_dot
    0           ...% 2 epos_dot
    0           ...% 3 alt_dot
    0           ...% 4 phi_dot
    0           ...% 5 theta_dot
    0           ...% 6 psi_dot
    5           ...% 7 U_dot
    100         ...% 8 V_dot
    100         ...% 9 W_dot
    10          ...% 10 P_dot
    10          ...% 11 Q_dot
    10          ...% 12 R_dot
    ];



%% Original init Values
% %% ------------------- Optimization initialization ------------------------
%
% % initial guess for the control inputs used as initial input for
% % optimization
% Trim.Controls_init.Thrust    = 3000;        % thrust in N
% Trim.Controls_init.Elevator  = -1;          % elevator in deg
% Trim.Controls_init.Rudder    = 0.01;        % rudder in deg
% Trim.Controls_init.Aileron   = 0.01;        % aileron in deg
% Trim.Controls_init.alpha     = 2 *(pi/180); % AoA in rad
% % dummy values used for trim routine size