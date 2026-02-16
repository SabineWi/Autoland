function [varargout] = fct_get_state_space_lat_var(LTI_full, state_idx, Trim,varargin)

%% -------------------------- Lateral Motion ------------------------------

nrSys = numel(varargin);
nrAlt = length(Trim.States.Alt);
nrVel = length(Trim.States.Vel);

for k = 1 : 1 : nrSys
    
    for i = 1 : 1 : nrAlt
        
        for j = 1 : 1 : nrVel
            
            if isequal('4x4', varargin{k})
                
                % eliminate states --> pure truncation, nothing more keeps state names
                % uses the state_idx of all states not necessary, it's easy to adress
                % them like that, the not specified ones (e.g. controller states are handled via state_idx.empty entry)
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.theta,...
                    state_idx.psi, state_idx.U, state_idx.W, state_idx.q, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_lat_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'r', 'V_body', 'p', 'phi'};
                [~, P] = ismember(LTI_lat_temp.StateName, desiredOrder);
                LTI_lat_temp = xperm(LTI_lat_temp, P);
                
                % select the outputs and inputs
                LTI_lat_temp = LTI_lat_temp([12, 30, 10, 4], [3, 4, 7]); % still hard coded as the in and outputs should not be changed
                set(LTI_lat_temp, 'Name', '4x4');
                
                
                
            elseif isequal('RollMode', varargin{k})
                
                % eliminate states --> pure truncation, nothing more keeps state names
                % uses the state_idx of all states not necessary, it's easy to adress
                % them like that, the not specified ones (e.g. controller states are handled via state_idx.empty entry)
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.theta,...
                    state_idx.psi, state_idx.U, state_idx.W, state_idx.V, state_idx.q, state_idx.r, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_lat_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'p', 'phi'};
                [~, P] = ismember(LTI_lat_temp.StateName, desiredOrder);
                LTI_lat_temp = xperm(LTI_lat_temp, P);
                
                % select the outputs and inputs
                LTI_lat_temp = LTI_lat_temp([10, 4], [3, 7]); % still hard coded as the in and outputs should not be changed
                set(LTI_lat_temp, 'Name', 'RollMode');
                
                
                
            elseif isequal('DutchRoll', varargin{k})
                
                % eliminate states --> pure truncation, nothing more keeps state names
                % uses the state_idx of all states not necessary, it's easy to adress
                % them like that, the not specified ones (e.g. controller states are handled via state_idx.empty entry)
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.phi, state_idx.theta,...
                    state_idx.psi,state_idx.U, state_idx.W, state_idx.p, state_idx.q, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_lat_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'r', 'V_body'};
                [~, P] = ismember(LTI_lat_temp.StateName, desiredOrder);
                LTI_lat_temp = xperm(LTI_lat_temp, P);
                
                % select the outputs and inputs
                LTI_lat_temp = LTI_lat_temp([12, 30], [4, 7]); % still hard coded as the in and outputs should not be changed
                set(LTI_lat_temp, 'Name', 'DutchRoll');
                
                
            elseif isequal('n_y', varargin{k})
                
                % eliminate states --> pure truncation, nothing more keeps state names
                % uses the state_idx of all states not necessary, it's easy to adress
                % them like that, the not specified ones (e.g. controller states are handled via state_idx.empty entry)
                elim = [state_idx.northpos, state_idx.eastpos, state_idx.Alt, state_idx.theta,...
                    state_idx.psi, state_idx.U, state_idx.W, state_idx.q, state_idx.lambda,...
                    state_idx.mue, state_idx.h, state_idx.thrust, state_idx.elevator,...
                    state_idx.aileron, state_idx.rudder, state_idx.LF_state, state_idx.LF_pos,...
                    state_idx.empty]; % states to eliminate
                
                % eliminate states
                LTI_lat_temp = modred(LTI_full(: ,:, i, j), elim, 'Truncate');
                
                % reorder the states
                desiredOrder = {'r', 'V_body', 'p', 'phi'};
                [~, P] = ismember(LTI_lat_temp.StateName, desiredOrder);
                LTI_lat_temp = xperm(LTI_lat_temp, P);
                
                % select the outputs and inputs
                LTI_lat_temp = LTI_lat_temp([12, 30, 10, 4, 26], [3, 4, 7]); % still hard coded as the in and outputs should not be changed
                set(LTI_lat_temp, 'Name', 'n_y');
                
                
                    
            else
                
                error(['The Setting ', varargin{k},' is not implemented', char(10)])
                
            end
            
            LTI_lat(:, :, i, j) = LTI_lat_temp;
            set(LTI_lat, 'Name', LTI_lat_temp.Name);
            
        end
        
    end
    
    varargout{k} = LTI_lat;
    clear LTI_lat
    
end


