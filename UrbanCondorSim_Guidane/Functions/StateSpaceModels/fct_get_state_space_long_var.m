function [varargout] = fct_get_state_space_long_var(LTI_full, state_idx, Trim, varargin)

% to do get rid of numerical inaccuracies for ovbvious 0 entries/
% coefficients

%% -------------------------- Longitudinal Motion -------------------------

nrSys = numel(varargin);
nrAlt = length(Trim.States.Alt);
nrVel = length(Trim.States.Vel);

for k = 1 : 1 : nrSys
    
    for i = 1 : 1 : nrAlt
        
        for j = 1 : 1 : nrVel
            
            if isequal('theta_4x4', varargin{k}) % theta is the long state coming from the linearization
                
                % eliminate states --> pure truncation, nothing more keeps state names
                % uses the state_idx of all states not necessary, it's easy to adress
                % them like that, the not specified ones (e.g. controller states are handled via state_idx.empty entry)               
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.phi,...
                    state_idx.psi, state_idx.V, state_idx.p, state_idx.r, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_long_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'U_body', 'theta', 'W_body', 'q'};
                [~, P] = ismember(LTI_long_temp.StateName, desiredOrder);
                LTI_long_temp = xperm(LTI_long_temp, P);
                
                % select the outputs and inputs
                LTI_long_temp = LTI_long_temp([28, 5, 29, 11], [1, 2, 6, 8]); % still hard coded as the in and outputs should not be changed
                set(LTI_long_temp, 'Name', 'theta_4x4');
                
                
% does not work since alpha is no state anymore                
%             elseif isequal('gamma_4x4', varargin{k})
%                 
%                 % eliminate states --> pure truncation, nothing more keeps state names
%                 elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.phi,...
%                     state_idx.psi, state_idx.V, state_idx.p, state_idx.r, state_idx.lambda,...
%                     state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
%                     state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
%                     state_idx.empty]; % states to eliminate
%                 
%                 % eliminate states
%                 LTI_long_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
%                 
%                 % reorder the states
%                 desiredOrder = {'U_body', 'theta', 'W_body', 'q'};
%                 [~, P] = ismember(LTI_long_temp.StateName, desiredOrder);
%                 LTI_long_temp = xperm(LTI_long_temp, P);
%                 
%                 % select the outputs and inputs
%                 LTI_long_temp = LTI_long_temp([28, 5, 29, 11], [1, 2, 6, 8]); % still hard coded as the in and outputs should not be changed
%                 
%                 % perform state transformation
%                 T_gam_theta = [1, 0, 0, 0; 0, 1, -1, 0; 0, 0, 1, 0; 0, 0, 0, 1];
%                 LTI_long_temp = ss2ss(LTI_long_temp, T_gam_theta);
%                 LTI_long_temp.StateName  = {'V', 'gamma', 'alpha', 'q'};
%                 LTI_long_temp.OutputName = {'V', 'gamma', 'alpha', 'q'};
%                 set(LTI_long_temp, 'Name', 'gamma_4x4');
                
                
                
            elseif isequal('ShortPeriod', varargin{k})
                
                % eliminate states --> pure truncation, nothing more keeps state names
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.phi, state_idx.theta,...
                    state_idx.psi, state_idx.U ,state_idx.V, state_idx.p, state_idx.r, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty];
                
                % eliminate states
                LTI_long_temp = modred(LTI_full(:, :, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'W_body', 'q'};
                [~, P] = ismember(LTI_long_temp.StateName, desiredOrder);
                LTI_long_temp = xperm(LTI_long_temp, P);
                
                % select the outputs and inputs
                LTI_long_temp = LTI_long_temp([29, 11], [2, 6, 8]); % still hard coded as the in and outputs should not be changed
                set(LTI_long_temp, 'Name', 'ShortPeriod');
                
                
                
            elseif isequal('Phugoid', varargin{k})
                
                % eliminate states --> pure truncation, nothing more keeps state names
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.phi,...
                    state_idx.psi , state_idx.V, state_idx.p,...
                    state_idx.r, state_idx.lambda, state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_long_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'U_body', 'theta', 'W_body', 'q'};
                [~, P] = ismember(LTI_long_temp.StateName, desiredOrder);
                LTI_long_temp = xperm(LTI_long_temp, P);
                
                % select the outputs and inputs
                LTI_long_temp = LTI_long_temp([28, 5, 29, 11], [1, 2, 6, 8]); % still hard coded as the in and outputs should not be changed
                
                % reduce to simple phugoid approx.
                elim_phu = [3:4]; % states to eliminate
                LTI_long_temp = modred(LTI_long_temp, elim_phu, 'Truncate');
                
                % select the phug. in- and outputs
                LTI_long_temp = LTI_long_temp([1,2], :);
                set(LTI_long_temp, 'Name', 'Phugoid');
                
                
                
            elseif isequal('n_x_z', varargin{k})
                
                % eliminate states --> pure truncation, nothing more keeps state names
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.phi,...
                    state_idx.psi, state_idx.V, state_idx.p, state_idx.r, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_long_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'U_body', 'theta', 'W_body', 'q'};
                [~, P] = ismember(LTI_long_temp.StateName, desiredOrder);
                LTI_long_temp = xperm(LTI_long_temp, P);
                
                % select the outputs and inputs
                LTI_long_temp = LTI_long_temp([28, 5, 29, 11, 25, 27], [1, 2, 6, 8]); % still hard coded as the in and outputs should not be changed
                set(LTI_long_temp, 'Name', 'n_x_z');
                
                
                
            elseif isequal('speedbrakes', varargin{k}) % theta is the long state coming from the linearization
                
                % eliminate states --> pure truncation, nothing more keeps state names
                % uses the state_idx of all states not necessary, it's easy to adress
                % them like that, the not specified ones (e.g. controller states are handled via state_idx.empty entry)               
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.phi,...
                    state_idx.psi, state_idx.V, state_idx.p, state_idx.r, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_long_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'U_body', 'theta', 'W_body', 'q'};
                [~, P] = ismember(LTI_long_temp.StateName, desiredOrder);
                LTI_long_temp = xperm(LTI_long_temp, P);
                
                % select the outputs and inputs
                LTI_long_temp = LTI_long_temp([28, 5, 29, 11], [1, 2, 5, 6, 8]); % still hard coded as the in and outputs should not be changed
                set(LTI_long_temp, 'Name', 'theta_4x4_SB');
                
                
                
            else
                
                error(['The Setting ', varargin{k},' is not implemented', char(10)])
                
            end
            
            LTI_long(:, :, i, j) = LTI_long_temp;
            set(LTI_long, 'Name', LTI_long_temp.Name);
            
        end
        
    end
    
    varargout{k} = LTI_long;
    clear LTI_long
end
