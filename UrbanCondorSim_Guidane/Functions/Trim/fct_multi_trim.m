function [Trim, state_idx] = fct_multi_trim(Trim, Aircraft, Environment)

%% Turn on Trim_Flag
set_param('Flight_Simulation/Flag_Settings/Trim_Flag','Value','1');


%% --------------------- Controls Init ------------------------------------

% - initial guess for the input variables for the trim optimization
% - also shows the free parameters for the optimization
% - initialised individually from InitializeAircraft_XX

u_trim_0 = [Trim.Controls_init.Thrust;...   % 1
            Trim.Controls_init.Elevator;... % 2
            Trim.Controls_init.Aileron;...  % 3
            Trim.Controls_init.Rudder;...   % 4
            Trim.Controls_init.alpha;...    % 5
            Trim.Controls_init.beta];       % 6


%% ------------------------ Compile simulink model ------------------------

Flight_Simulation([], [], [], 'compile')
            

%% ------------------------ Get integrator states -------------------------
% identify the states of the simulink model and get their index

[state_idx] = fct_get_simulink_mdl_state(u_trim_0, Aircraft, Environment.Wind_Input_UVW);


%% ------------------------ Optimization settings -------------------------

opt = optimset('TolFun', 1e-10, 'TolX', 1e-10, 'MaxFunEvals', 5e+04, 'MaxIter', 1e+04);

iterate   = 1; % on
counter     = 0;


%% ------------------------- Actual trim routine --------------------------

nrAlt = length(Trim.States.Alt);
nrVel = length(Trim.States.Vel);

for i = 1 : 1 : nrAlt
    
    for j = 1 : 1 : nrVel
        
        while isequal(iterate, 1)
            
            fct_trim_handle = @(u) fct_multi_trim_generic(u, Trim, state_idx, i, j, Aircraft, Environment);
            
            
            tic
            
            [u_trim,FVAL,EXITFLAG,OUTPUT] = fminsearch(fct_trim_handle,u_trim_0, opt); % trim function must exist, it's the called function handle
            % trimfun is the actual trim function handle which gets called by the fminsearch
            
            [cost, Xdot, trim_results] = fct_trim_handle(u_trim);
            
            toc
            
            
            counter = counter + 1;
            
            
            disp([char(10),'Trim Values and Cost:'])
            disp(['cost   = ' num2str(cost)])
            disp(['thrust = ' num2str(trim_results(state_idx.thrust)) ' N'])
            disp(['elev   = ' num2str(trim_results(state_idx.elevator)) ' deg'])
            disp(['ail    = ' num2str(trim_results(state_idx.aileron)) ' deg'])
            disp(['rud    = ' num2str(trim_results(state_idx.rudder)) ' deg'])
            disp(['alpha  = ' num2str(atan( (trim_results(state_idx.W)-Environment.Wind_Input_UVW(3)) / (trim_results(state_idx.U)-Environment.Wind_Input_UVW(1) ) ) *180/pi) ' deg'])
            disp(['beta   = ' num2str(atan( (trim_results(state_idx.V)-Environment.Wind_Input_UVW(2)) / (trim_results(state_idx.U)-Environment.Wind_Input_UVW(1) ) ) *180/pi) ' deg'])
            disp(['Number of execution = ', num2str(counter)])

            
            if cost < 1e-24 || counter > 10 % was 1e-29
                iterate = 0; % off - stop iterating
            end
            % Remarks:
            % - Everything below 1e-10 should be sufficient according to
            %   Stevens&Lewis p.191
            % - Further decrease is mainly for checking consistency
            
            u_trim_0 = u_trim; % set previous optimization results as new init values to improve performance
            
        end
        
        disp([char(10), 'Selected Aircraft: ', Aircraft.Name])
        
        % Select a name of the selected flight condition
        if Trim.States.Gam == 0 && Trim.States.Psidot == 0
            Trim.FlightCondition = 'Steady horizontal flight';
            
        elseif Trim.States.Gam < 0 && Trim.States.Psidot == 0
            Trim.FlightCondition = 'Steady descending flight';
            
        elseif Trim.States.Gam > 0 && Trim.States.Psidot == 0
            Trim.FlightCondition = 'Steady ascending flight';          
            
        elseif Trim.States.Gam == 0 && abs(Trim.States.Psidot) > 0
            Trim.FlightCondition = 'Steady coordinated turn';
            
        elseif Trim.States.Gam < 0 && abs(Trim.States.Psidot) > 0
            Trim.FlightCondition = 'Steady descending turn';
            
        elseif Trim.States.Gam > 0 && abs(Trim.States.Psidot) > 0
            Trim.FlightCondition = 'Steady ascending turn';
            
        elseif Trim.States.Nz_delta > 0
            Trim.FlightCondition = 'Steady symmetric pull-up maneuver';
            
        elseif Trim.States.Nz_delta < 0
            Trim.FlightCondition = 'Steady symmetric push-over maneuver';

        end 
        
        % display name of the trimmed flight condition
        disp(['Trimmed for: ', Trim.FlightCondition])
        
        % Display trim point variables
        disp(['With: Alt = ', num2str(Trim.States.Alt(i)),' m,  V = ', num2str(Trim.States.Vel(j)), ' m/s,  gamma = ', num2str(Trim.States.Gam),' deg,  psi_dot = ', num2str(Trim.States.Psidot), ' deg/s,  D_n_z = ', num2str(Trim.States.Nz_delta), ' g'])

        if counter <= 10            
                disp(['<strong>Trim successfull</strong>'])            
        else
                disp(['<strong>Trim unsuccessfull</strong>'])            
        end        
        
        % -----------------------------------------------------------------
        % -------------------------- Results ------------------------------
        % -----------------------------------------------------------------
        
        % -------------------------- Controls -----------------------------
        
        % Easy access to the trim results -> control inputs
        
        Trim.Controls_result.Thrust(i, j)     = trim_results(state_idx.thrust);
        Trim.Controls_result.Elevator(i, j)   = trim_results(state_idx.elevator);
        Trim.Controls_result.Aileron(i, j)    = trim_results(state_idx.aileron);
        Trim.Controls_result.Rudder(i, j)     = trim_results(state_idx.rudder);
        
        Trim.Input_result_vec{i, j}           = trim_results([state_idx.thrust, state_idx.elevator, state_idx.aileron, state_idx.rudder]);
        
        
        % ---------------------------- Flaps --------------------------------------
        
        % include Flaps if necessary ... only necessary if the Flaps have a
        % scheduled behavior with respect to other states which may have to be
        % recognized
        
        % -------------------------- States ---------------------------------------
        % Easy and traceable access to the trim results -> states
        
        % ----- Position
        
        % NED frame
        Trim.States_result.north_pos(i, j)    = trim_results(state_idx.northpos);
        Trim.States_result.east_pos(i, j)     = trim_results(state_idx.eastpos);
        Trim.States_result.Altitude(i, j)     = trim_results(state_idx.Alt);
        
        % WGS84 (uses the trim states as it has no influence but keeps the units consistent for simulink)
        Trim.States_result.lambda(i, j)       = Trim.States.lambda;
        Trim.States_result.mue(i, j)          = Trim.States.mue;
        Trim.States_result.h(i)               = Trim.States.h(i);
        
        % ----- Attitude
        Trim.States_result.phi(i, j)          = trim_results(state_idx.phi);
        Trim.States_result.theta(i, j)        = trim_results(state_idx.theta);
        Trim.States_result.psi(i, j)          = trim_results(state_idx.psi);
        
        % ----- Translation
        Trim.States_result.U(i, j)            = trim_results(state_idx.U);
        Trim.States_result.V(i, j)            = trim_results(state_idx.V);
        Trim.States_result.W(i, j)            = trim_results(state_idx.W);
        
        % ----- Rotation
        Trim.States_result.p(i, j)            = trim_results(state_idx.p);
        Trim.States_result.q(i, j)            = trim_results(state_idx.q);
        Trim.States_result.r(i, j)            = trim_results(state_idx.r);
        
        
        % --------------------- States Derivative Vector --------------------------
        % easy check if the trim results are sufficient
        Trim.States_results_dot_vec{i, j}     = Xdot;
        
        % -------------------------- States Vector --------------------------------
        % Used for the state integrator initialization
        Trim.States_result_vec{i, j}          = trim_results(1:12);
        
        
        % reset the conditions of the while command
        
        counter = 0;
        iterate = 1; % iterate back to "on"
        
    end
    
end

%% ------------------------- Terminate Simulink Model ---------------------

Flight_Simulation([], [], [], 'term')


%% Turn off Trim_Flag
set_param('Flight_Simulation/Flag_Settings/Trim_Flag','Value','0');


end

