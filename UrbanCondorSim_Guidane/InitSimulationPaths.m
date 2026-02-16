%% ------------------------ Functions folder ------------------------------
% containing all functions files needed for trim, linearization etc.

% trim functions
addpath('.\Functions\Trim')
addpath('.\Functions\Trim\Templates')

% Linearization
addpath('.\Functions\Linearization')

% Prepare simulink
addpath('.\Functions\PrepareSimulink')

% create state space models
addpath('.\Functions\StateSpaceModels')

%% ----------------------- Init files folder ------------------------------
% containing all initialisation files

% To initialize Aircraft
addpath('.\InitFiles\InitAircraft')

% For the aerodynamic look up tables
addpath('.\InitFiles\Aero_LookUp_Tables')

% Initialize required constant blocks e.g. trim values (dummy values)
addpath('.\InitFiles\InitSimulink')


%% ------------------------- Images folder --------------------------------

addpath('.\Images')

%% --------------------- Flight Control Design ----------------------------
% Contains scripts for control design and initialisation

addpath('.\FlightControlDesign')


%% ----------------------- Set-up Environment -----------------------------

addpath('.\Environment')

%% -------------------------- Guidance ------------------------------------
addpath('.\GeneratePathData')
