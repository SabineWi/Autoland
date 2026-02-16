function [LTI_full] = fct_multi_lin(Trim, Environment, Aircraft, state_idx)


%% linearisation for each trim point

% Get the number of trim velocities and altitudes
nrAlt = length(Trim.States.Alt);
nrVel = length(Trim.States.Vel);

for i = 1 : 1 : nrAlt
    
    for j = 1 : 1 : nrVel
        
        U_in.time           = 0; % just one execution
        
        % control surfaces and wind inputs with the following order:
        % 1. thrust, 2. ele, 3. ail, 4. rud, 5. SB, 6. Wind_U, 7. Wind_V, 7. Wind_W
        U_in.signals.values = [Trim.Input_result_vec{i, j}; Aircraft.Speedbrakes; Environment.Wind_Input_UVW];
        
        % ------------------- get and set the input port sizes ------------
        
        Dim_U_in = get_param('Flight_Simulation/U_in', 'PortDimensions');
        
        if isequal(str2num(Dim_U_in), length(U_in.signals.values))
            
        else
            set_param('Flight_Simulation/U_in', 'PortDimensions', num2str(length(U_in.signals.values)))
        end
        
    
        [sys,x0,str,ts] = Flight_Simulation(0, [], [], 'sizes');
        
        if ~isequal(str, state_idx.str(:, 1))
            error(['The state order does not match the pre-allocated case', char(10)])
        end
        
        
        
        % ------------- create the initial states array -------------------
        
        states = x0;
        
        % ---------------------- A/C states - flight mechanic -------------
        
        % Attitude
        states(state_idx.phi)       = Trim.States_result.phi(i, j);
        states(state_idx.theta)     = Trim.States_result.theta(i, j);
        states(state_idx.psi)       = Trim.States_result.psi(i, j);
        
        % Translation
        states(state_idx.U)         = Trim.States_result.U(j);
        states(state_idx.V)         = Trim.States_result.V(i, j);
        states(state_idx.W)         = Trim.States_result.W(i, j);
        
        % Rates
        states(state_idx.p)         = Trim.States_result.p(i, j);
        states(state_idx.q)         = Trim.States_result.q(i, j);
        states(state_idx.r)         = Trim.States_result.r(i, j);
        
        
        % ------------------------- A/C states - position -----------------
        
        % NED frame
        states(state_idx.northpos)  = Trim.States_result.north_pos(i, j);
        states(state_idx.eastpos)   = Trim.States_result.east_pos(i, j);
        states(state_idx.Alt)       = Trim.States_result.Altitude(i);
        
        % WGS84 states (uses the trim states as it has no influence but keeps the units consistent)
        states(state_idx.mue)       = Trim.States.mue * (pi/180); % mue WGS84
        states(state_idx.lambda)    = Trim.States.lambda * (pi/180); % lambda WGS84
        states(state_idx.h)         = Trim.States.h(i); % altitude WGS84
        
        % -------------------------- Actuator states ----------------------
        
        % Actuator states
        states(state_idx.thrust)    = Trim.Controls_result.Thrust(i, j); % thrust
        states(state_idx.elevator)  = Trim.Controls_result.Elevator(i, j); % elevator
        states(state_idx.aileron)   = Trim.Controls_result.Aileron(i, j); % aileron
        states(state_idx.rudder)    = Trim.Controls_result.Rudder(i, j); % rudder
        
        % Leading edge states
        states(state_idx.LF_state)  = -(180/pi)*atan( (Trim.States_result.W(i, j)-Environment.Wind_Input_UVW(3)) / (Trim.States_result.U(i, j)-Environment.Wind_Input_UVW(1) ) ); % -(180/pi)*alpha
        states(state_idx.LF_pos)    = -(180/pi)*atan( (Trim.States_result.W(i, j)-Environment.Wind_Input_UVW(3)) / (Trim.States_result.U(i, j)-Environment.Wind_Input_UVW(1) ) ); % -(180/pi)*alpha
        
        % --------------------- create inputs array -----------------------
        
        % first 4 are the commands:
        inputs(1) = Trim.Controls_result.Thrust(i, j); % thrust
        inputs(2) = Trim.Controls_result.Elevator(i, j); % elevator
        inputs(3) = Trim.Controls_result.Aileron(i, j); % aileron
        inputs(4) = Trim.Controls_result.Rudder(i, j); % rudder
        inputs(5) = Aircraft.Speedbrakes;
        
        % next 3 are external wind inputs:
        inputs(6) = Environment.Wind_Input_UVW(1);
        inputs(7) = Environment.Wind_Input_UVW(2);
        inputs(8) = Environment.Wind_Input_UVW(3);
        
        % --------------------- run linearisation -------------------------
        
        set_param('Flight_Simulation','InitInArrayFormatMsg', 'None')
        
        % Turn on Lin_Flag
        set_param('Flight_Simulation/Flag_Settings/Lin_Flag','Value','1');
        
        % Call Matlab's built-in linearisation function
        % Obtains system matrices
        [A, B, C, D] = linmod('Flight_Simulation', states, inputs);
        
        % Turn off Lin_Flag
        set_param('Flight_Simulation/Flag_Settings/Lin_Flag','Value','0');
        
        % Build state space model from system matrices
        LTI_full(:, :, i,j) = ss(A, B, C, D);
        
        set_param('Flight_Simulation','InitInArrayFormatMsg', 'Warning')
        
    end
    
end


%% Naming of the states, inputs and outputs

% ------------------------- Set the input names ---------------------------

% hard coded atm
LTI_full.InputName  = {'Thrust', 'Elev', 'Ail', 'Rud', 'SB', 'Wind_U', 'Wind_V', 'Wind_W'};

% ------------------------- Set the state names ---------------------------
% note for improvement: wouldn't it be much easier to assign state names to
% every state within their Simulink field and use these?

% remove everything besides the integrator name from the state_idx.str
for ii = 1 : 1 : numel(state_idx.str(:, 1))

    idx_int = find(ismember(state_idx.str{ii, 1}, '/'), 1, 'last');
    state_idx.str_int(ii, 1) = {state_idx.str{ii, 1}(idx_int+1 : end)};

end

% get all state names from Simulink (integrator names)
LTI_full.StateName = state_idx.str_int(:, 1);

% ----- change relevant state names

% Position
LTI_full.StateName(state_idx.northpos)  = {'north_pos'};
LTI_full.StateName(state_idx.eastpos)   = {'east_pos'};
LTI_full.StateName(state_idx.Alt)       = {'Altitude'};

% Attitude
LTI_full.StateName(state_idx.phi)       = {'phi'};
LTI_full.StateName(state_idx.theta)     = {'theta'};
LTI_full.StateName(state_idx.psi)       = {'psi'};

% Translation
LTI_full.StateName(state_idx.U)         = {'U_body'};
LTI_full.StateName(state_idx.V)         = {'V_body'};
LTI_full.StateName(state_idx.W)         = {'W_body'};

% Rotation
LTI_full.StateName(state_idx.p)         = {'p'};
LTI_full.StateName(state_idx.q)         = {'q'};
LTI_full.StateName(state_idx.r)         = {'r'};

% atm there is no intention to change this as the outputs in simulink
% should not be changed cmp. trim routine
LTI_full.OutputName = {'north_pos', 'east_pos', 'Altitude',...
                       'phi', 'theta', 'psi',...
                       'U_body', 'V_body', 'W_body',...
                       'p', 'q', 'r',...%12
                       'north_pos_dot', 'east_pos_dot', 'Altitude_dot',...
                       'phi_dot', 'theta_dot', 'psi_dot',...
                       'U_body_dot', 'V_body_dot', 'W_body_dot',...
                       'p_dot', 'q_dot', 'r_dot',...%24
                       'n_x', 'n_y', 'n_z',...
                       'V_aero', 'alpha', 'beta'};%30


end
