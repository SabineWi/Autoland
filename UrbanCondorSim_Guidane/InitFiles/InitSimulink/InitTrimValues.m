% -------------------------- Controls -------------------------------------
% initializes the integrator x_0 states to prepare the system for compiling
% 
Trim.Controls_result.Thrust     = 0;
Trim.Controls_result.Elevator   = 0;
Trim.Controls_result.Aileron    = 0;
Trim.Controls_result.Rudder     = 0;


% -------------------------- States ---------------------------------------
% Easy and traceable access to the trim results -> states

% Position
Trim.States_result.north_pos    = 0;
Trim.States_result.east_pos     = 0;
Trim.States_result.Altitude     = 0;

Trim.States_result.lambda       = 0;
Trim.States_result.mue          = 0;        
Trim.States_result.h            = 0;

% Attitude
Trim.States_result.phi          = 0;
Trim.States_result.theta        = 0;
Trim.States_result.psi          = 0;

% Translation
Trim.States_result.U            = 10;
Trim.States_result.V            = 0;
Trim.States_result.W            = 0;

% Rotation
Trim.States_result.p            = 0;
Trim.States_result.q            = 0;
Trim.States_result.r            = 0;

