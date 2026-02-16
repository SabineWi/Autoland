function [state_idx] = fct_get_simulink_mdl_state(u_trim_0, Aircraft, Wind_Input_UVW)


U_in.time           = 0; % just one execution

% control surfaces and wind inputs with the following order:
% .................... 1. thrust .. 2. ele ..... 3. ail ..... 4. rud ..... 5. Speedbrakes........6. Wind_U .. 7. Wind_V .. 8. Wind_W
U_in.signals.values = [u_trim_0(1), u_trim_0(2), u_trim_0(3), u_trim_0(4), Aircraft.Speedbrakes, Wind_Input_UVW'];

% ------------------- get and set the inport port sizes -------------------

% put this into the trim routine or whereever to increase the performance
Dim_U_in = get_param('Flight_Simulation/U_in', 'PortDimensions');

if isequal(str2num(Dim_U_in), length(U_in.signals.values))
    
else
    set_param('Flight_Simulation/U_in', 'PortDimensions', num2str(length(U_in.signals.values)))
end


% identify the states of the simulink model and store their positions
% so that new states included via the controller etc can be identified
[sys,x0,str,ts] = Flight_Simulation(0, [], [], 'sizes');

% ------------------------------- A/C states ------------------------------


% north position
state_idx.northpos = find(contains(str, 'npos_int'));
str{find(contains(str(:,1), 'npos_int')),2} = find(contains(str(:,1), 'npos_int'));

% east position
state_idx.eastpos  = find(contains(str(:, 1), 'epos_int'));
str{find(contains(str(:,1), 'epos_int')),2} = find(contains(str(:,1), 'epos_int'));

% Alt
state_idx.Alt = find(contains(str(:, 1), 'alt_int'));
str{find(contains(str(:,1), 'alt_int')),2} = find(contains(str(:,1), 'alt_int'));

% phi
state_idx.phi = find(contains(str(:, 1), 'phi_int'));
str{find(contains(str(:,1), 'phi_int')),2} = find(contains(str(:,1), 'phi_int'));

% theta
state_idx.theta = find(contains(str(:, 1), 'theta_int'));
str{find(contains(str(:,1), 'theta_int')),2} = find(contains(str(:,1), 'theta_int'));

% psi
state_idx.psi = find(contains(str(:, 1), 'psi_int'));
str{find(contains(str(:,1), 'psi_int')),2} = find(contains(str(:,1), 'psi_int'));

% V
state_idx.U = find(contains(str(:, 1), 'U_int'));
str{find(contains(str(:,1), 'U_int')),2} = find(contains(str(:,1), 'U_int'));

% alpha
state_idx.V = find(contains(str(:, 1), 'V_int'));
str{find(contains(str(:,1), 'V_int')),2} = find(contains(str(:,1), 'V_int'));

% beta
state_idx.W = find(contains(str(:, 1), 'W_int'));
str{find(contains(str(:,1), 'W_int')),2} = find(contains(str(:,1), 'W_int'));

% p
state_idx.p = find(contains(str(:, 1), 'p_int'));
str{find(contains(str(:,1), 'p_int')),2} = find(contains(str(:,1), 'p_int'));

% q
state_idx.q = find(contains(str(:, 1), 'q_int'));
str{find(contains(str(:,1), 'q_int')),2} = find(contains(str(:,1), 'q_int'));

% r
state_idx.r = find(contains(str(:, 1), 'r_int'));
str{find(contains(str(:,1), 'r_int')),2} = find(contains(str(:,1), 'r_int'));

% lambda
state_idx.lambda = find(contains(str(:, 1), 'lambda_int'));
str{find(contains(str(:,1), 'lambda_int')),2} = find(contains(str(:,1), 'lambda_int'));

% mue
state_idx.mue = find(contains(str(:, 1), 'mue_int'));
str{find(contains(str(:,1), 'mue_int')),2} = find(contains(str(:,1), 'mue_int'));

% h
state_idx.h   = find(contains(str(:, 1), 'h_int'));
str{find(contains(str(:,1), 'h_int')),2} = find(contains(str(:,1), 'h_int'));

% ------------------------------- Actuators -------------------------------

% thrust
state_idx.thrust = find(contains(str(:, 1), 'Thrust_pos_int'));
str{find(contains(str(:,1), 'Thrust_pos_int')),2} = find(contains(str(:,1), 'Thrust_pos_int'));

% elevator
state_idx.elevator = find(contains(str(:, 1), 'Elev_pos_int'));
str{find(contains(str(:,1), 'Elev_pos_int')),2} = find(contains(str(:,1), 'Elev_pos_int'));

% aileron
state_idx.aileron = find(contains(str(:, 1), 'Ail_pos_int'));
str{find(contains(str(:,1), 'Ail_pos_int')),2} = find(contains(str(:,1), 'Ail_pos_int'));

% rudder
state_idx.rudder = find(contains(str(:, 1), '/Rud_pos_int'));
str{find(contains(str(:,1), 'Rud_pos_int')),2} = find(contains(str(:,1), 'Rud_pos_int'));

% LF
% state
state_idx.LF_state = find(contains(str(:, 1), 'LF_state'));
str{find(contains(str(:,1), 'LF_state')),2} = find(contains(str(:,1), 'LF_state'));

% pos
state_idx.LF_pos = find(contains(str(:, 1), 'LF_pos_int'));
str{find(contains(str(:,1), 'LF_pos_int')),2} = find(contains(str(:,1), 'LF_pos_int'));


% find the states which are not necessary for the trim/lin and therefore
% have an empty entry in the second column
state_idx.empty = find(cellfun('isempty', str));

state_idx.str = str;