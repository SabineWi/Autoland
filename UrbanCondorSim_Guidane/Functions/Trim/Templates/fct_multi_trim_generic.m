function [cost, Xdot, states] = fct_multi_trim_generic(u, Trim, state_idx, nrAlt, nrVel, Aircraft, Environment)

% values, that are input externally from the TrimAndLin script to define
% the desired flight condition are defined with capital letter
% e.g. Trim.States.Vel is defined externally while Trim.States.vel is
% defined inside this template

%% --------------------- Initialise trim states  --------------------------

Trim.States.npos    = 0;                       % north pos [m]
Trim.States.epos    = 0;                       % east pos  [m]
Trim.States.alt     = Trim.States.Alt(nrAlt);  % altitude [m]

Trim.States.phi     = 0; % [rad] adjusted in coordinated turn constraint
Trim.States.theta   = 0; % [rad] adjusted in rate of climb constraint
Trim.States.psi     = 0;  % [rad] - start flying north

Trim.States.vel     = Trim.States.Vel(nrVel); % [m/s]
Trim.States.alpha   = u(5);                   % [rad]
Trim.States.beta    = u(6);                   % [rad]

Trim.States.p       = 0; % [rad/s]
Trim.States.q       = 0; % [rad/s] adjusted in turn-rates constraint
Trim.States.r       = 0; % [rad/s] adjusted in turn-rates constraint


%% ---------------- Set limits for trim variables -------------------------
% get limits from "Aircraft", initialised individually in "InitAircraft"

if u(1) > Aircraft.Engine.thrust_max
    u(1) = Aircraft.Engine.thrust_max;
elseif u(1) < Aircraft.Engine.thrust_min
    u(1) = Aircraft.Engine.thrust_min;
end

if u(2) > Aircraft.Controls.Elev_pos_max
    u(2) = Aircraft.Controls.Elev_pos_max;
elseif u(2) < Aircraft.Controls.Elev_pos_min
    u(2) = Aircraft.Controls.Elev_pos_min;
end

if u(3) > Aircraft.Controls.Ail_pos_max
    u(3) = Aircraft.Controls.Ail_pos_max;
elseif u(3) < Aircraft.Controls.Ail_pos_min
    u(3) = Aircraft.Controls.Ail_pos_min;
end

if u(4) > Aircraft.Controls.Rud_pos_max
    u(4) = Aircraft.Controls.Rud_pos_max;
elseif u(4) < Aircraft.Controls.Rud_pos_min
    u(4) = Aircraft.Controls.Rud_pos_min;
end

if Trim.States.alpha > Aircraft.AoA_Limits.max*pi/180
    Trim.States.alpha = Aircraft.AoA_Limits.max*pi/180;
elseif Trim.States.alpha < Aircraft.AoA_Limits.min*pi/180
    Trim.States.alpha = Aircraft.AoA_Limits.min*pi/180;
end

if Trim.States.beta > Aircraft.Sideslip_Limits.max*pi/180
    Trim.States.beta = Aircraft.Sideslip_Limits.max*pi/180;
elseif Trim.States.beta < Aircraft.Sideslip_Limits.min*pi/180
    Trim.States.beta = Aircraft.Sideslip_Limits.min*pi/180;
end


%% ----------- Constraints due to desired flight conditions ---------------

% pre-calculations
sGam    = sin( Trim.States.Gam * (pi/180) );
sAlpha  = sin( Trim.States.alpha );
cAlpha  = cos( Trim.States.alpha );
tAlpha  = tan( Trim.States.alpha );
sBeta   = sin( Trim.States.beta );
cBeta   = cos( Trim.States.beta );
g0 = Environment.Const.g0;

% pull-up (symmetric)
Trim.States.thetadot = g0 ./ Trim.States.vel .* (Trim.States.Nz_delta);

% turn coordination (allowing simultaneous climb)
Trim.States.psidot  = Trim.States.Psidot * pi/180; % [rad/s]
gc = Trim.States.psidot * Trim.States.vel ./ g0;
a = 1 - gc*tAlpha*sBeta;
b = sGam/cBeta;
c = 1 + gc*gc*cBeta*cBeta;
Trim.States.phi = atan((gc*cBeta*((a-b*b)+...
                        b*tAlpha*sqrt(c*(1-b*b)+gc*gc*sBeta*sBeta)))/...
                       (cAlpha*(a*a-b*b*(1+c*tAlpha*tAlpha))));
                   
% rate of climb
aa = cAlpha*cBeta;
bb = sin(Trim.States.phi)*sBeta+cos(Trim.States.phi)*sAlpha*cBeta;
Trim.States.theta = atan((aa*bb+sGam*sqrt(aa^2-sGam*sGam+bb^2))/(aa^2-sGam*sGam));

Trim.States.phidot = 0; % could be given as an external parameter from 
                        % TrimAndLin to trim a constant roll rate

% turn rates
if Trim.States.phidot ~= 0 % rolling
    
    Trim.States.p = Trim.States.phidot;
    Trim.States.r = Trim.States.phidot*tAlpha;
    
elseif Trim.States.psidot ~= 0 % yawing
    
    Trim.States.p = -Trim.States.psidot*sin(Trim.States.theta);
    Trim.States.q = Trim.States.psidot*sin(Trim.States.phi)*cos(Trim.States.theta);
    Trim.States.r = Trim.States.psidot*cos(Trim.States.phi)*cos(Trim.States.theta);
    
elseif Trim.States.thetadot ~= 0 % pitching
    
    Trim.States.q = Trim.States.thetadot;
    
end


%% Set simulink input vector U_in for Simulation

U_in.time           = 0; % just one execution

% control surfaces and wind inputs with the following order:
% 1. thrust, 2. ele, 3. ail, 4. rud, 5. Speedbrakes, 6. Wind_U, 7. Wind_V, 8. Wind_W
U_in.signals.values = [u(1), u(2), u(3), u(4), Aircraft.Speedbrakes, Environment.Wind_Input_UVW'];


%% ----------------- get and set the input port sizes ---------------------

Dim_U_in = get_param('Flight_Simulation/U_in', 'PortDimensions');

if ~isequal(str2num(Dim_U_in), length(U_in.signals.values))
    set_param('Flight_Simulation/U_in', 'PortDimensions', num2str(length(U_in.signals.values)))
end


%% ----------------- create the initial states array ----------------------

[sys,x0,str,ts] = Flight_Simulation(0, [], [], 'sizes');

states = x0;

% check order of states array
if ~isequal(str, state_idx.str(:, 1))
   error(['The state order does not match the pre-allocated case', char(10)])
end


%% --------------------- A/C states - flight mechanic ---------------------
% depending on desired flight condition

% Attitude
states(state_idx.phi)     = Trim.States.phi;
states(state_idx.theta)   = Trim.States.theta;
states(state_idx.psi)     = Trim.States.psi;

% Translation
% e.g. U_body = V_aero*cos(alpha)*cos(beta) + U_wind
states(state_idx.U)       = Trim.States.vel*cAlpha*cBeta + Environment.Wind_Input_UVW(1);
states(state_idx.V)       = Trim.States.vel*sBeta + Environment.Wind_Input_UVW(2);
states(state_idx.W)       = Trim.States.vel*sAlpha*cBeta + Environment.Wind_Input_UVW(3);

% Rates
states(state_idx.p)       = Trim.States.p;
states(state_idx.q)       = Trim.States.q;
states(state_idx.r)       = Trim.States.r;

%% ------------------------ A/C states - position -------------------------

% NED frame
states(state_idx.northpos)  = Trim.States.npos;
states(state_idx.eastpos)   = Trim.States.epos;
states(state_idx.Alt)       = Trim.States.alt;

% WGS84 states
states(state_idx.mue)       = Trim.States.mue * (pi/180); % mue WGS84
states(state_idx.lambda)    = Trim.States.lambda * (pi/180); % lambda WGS84
states(state_idx.h)         = Trim.States.alt; % altitude WGS84

%% ------------------------- Actuator states ------------------------------

% Actuator states
states(state_idx.thrust)    = u(1);
states(state_idx.elevator)  = u(2);
states(state_idx.aileron)   = u(3);
states(state_idx.rudder)    = u(4);

% Leading edge Flap states
states(state_idx.LF_state)  = -(180/pi)*Trim.States.alpha; % alpha dependant
states(state_idx.LF_pos)    = -(180/pi)*Trim.States.alpha;
% only for the F16 model and not yet fully implemented in Simulink

% ---------------------- States not in use --------------------------------
% this identifies the states which are not used for the trim at all, this
% includes the controller states etc.

% if ~isempty(state_idx.empty)
%     states(state_idx.empty) = 0;
% else
% end

% It's not in use right now as all states array gets
% pre-allocated with the data at t = 0, before the trim states are set


%% -------------------- Create inputs array -------------------------------

% first 4 are the control commands:
inputs(1) = u(1); % thrust
inputs(2) = u(2); % elevator
inputs(3) = u(3); % aileron
inputs(4) = u(4); % rudder
inputs(5) = Aircraft.Speedbrakes;

% next 3 are external wind inputs:
inputs(6) = Environment.Wind_Input_UVW(1); % Wind_U
inputs(7) = Environment.Wind_Input_UVW(2); % Wind_V
inputs(8) = Environment.Wind_Input_UVW(3); % Wind_W


%% --------------------- obtain system responses --------------------------

set_param('Flight_Simulation','InitInArrayFormatMsg', 'None')

outputs = Flight_Simulation(0, states, inputs, 'outputs');
% derivs  = F16Block_alpha_new(0,states,inputs,'derivs'); % maybe use
% later to make it more flexible in combination with outsourcing states
% names etc... make it more robust with respect to the controller
% implementation

set_param('Flight_Simulation','InitInArrayFormatMsg', 'Warning')


%% ------------------ Calculate costs from Derivates ----------------------

% Obtain derivates vector from system response
Xdot = outputs(13:24);
% Order of derivatives in Xdot is:
% 1 npos_dot, 2 epos_dot,  3 alt_dot,  4 phi_dot, 5 theta_dot, 6 psi_dot
% 7 U_dot,    8 V_dot,     9 W_dot,    10 P_dot,  11 Q_dot,    12 R_dot


% --------------------- Create weight function ----------------------------

% - weight = 0 means that it's a free variable and can be arbitrary without
%   violating the trim condition
% - weights have rather small impact on calculation speed and solution
% - they are initialised in the aircraft initialisation scripts

weight = Aircraft.Weight_for_OptCost;

% calculate costs
cost = weight*(Xdot.^2);


end
