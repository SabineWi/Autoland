%% ------------------ Wind Settings Script - Remarks ----------------------
% Remarks:
% Wind settings can be executed any time without rerunning TrimAndLin
% It is run once during TrimAndLin though, for initialisation

% Important:
% Wind velocities are defined in the NED earth linked coordinate system
% -> z is downwards
% -> x,y are positive in north and east direction, respectively

% A turbulence model has not yet been implemented


%% --------------- Constant Wind Input Velocities [U,V,W] -----------------
% - Inputs a constant! wind velocity through the model input U_in
% - To be used especially for trimming and linearizing in a specific,
%   constant wind velocity
%   -> It is only used during TrimAndLin, where the proper time dependant
%      wind model is bypassed
%   -> Inserts a wind input to the state space models (LTIs)
% - Set to 0 by default

Environment.Wind_Input_UVW  = [0;0;0]; % [m/s]


%% -------------------- Wind Model for Simulation -------------------------
% Proper time dependant wind model for the simulation in Simulink


% ------------------------ Gust activation Settings -----------------------

% Turn on 1-cos gust in [U,V,W] direction, amplitude is set in 1-cos settings
Environment.gust.flags         = [0,0,0]; % 0 = off, 1 = on, -1 = negative direction

% Turn on simple step gusts instead - in [U,V,W] direction - by setting amplitude
Environment.gust.step_amplitude = [0,0,0]; % [m/s], any value ~=0 turns it on

% Time after which gusts in [U,V,W] direction start
Environment.gust.steptimes     = [1,1,5]; % [s]

% ----------------------- 1-cos gust shape settings -----------------------

% Set [U,V,W] gradient length "H"
% Suggestion: use H = 12.5 * Aircraft.Dimensions.MAC
Environment.gust.gradientlength(1:3) = 12.5 .* Aircraft.Dimensions.MAC; % [m]
% or set manually
% Environment.gust.gradientlength    = [50,50,1]; % [m]
% According to CS-25 H should range from 9.1m to 107m

% Switch between automatic calculation of the gust amplitude according to
% CS-25 standards or manual amplitude selection
Environment.gust.switch_auto           = 0; % 1 for auto, 0 for manual

% For manual gust amplitude selection:
% Set [U,V,W] amplitude manually
Environment.gust.man_amplitude         = [10,10,5]; % [m/s]



% ---------------------- Turbulence Model Settings ------------------------
% Turbulence model has yet to be implemented

% ACSP.TURBW.WX33=WX33;
% ACSP.TURBW.WY33=WY33;
% ACSP.TURBW.sigu=0.15*sqrt(WX33^2+WY33^2);
% ACSP.TURBW.sigw=0.77;
% ACSP.TURBW.seedwx=1;
% ACSP.TURBW.seedwy=2;
% ACSP.TURBW.seedwz=3;
