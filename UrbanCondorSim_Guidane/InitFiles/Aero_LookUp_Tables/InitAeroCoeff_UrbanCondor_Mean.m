% for the calculation of the derivatives AVL was used, with:
% - body-axis derivatives (sb) --> aircraft coordinate system
% - stability derivatives (st) --> flow coordinate system 
% Runcase: 
% - Ca=0.7 (old: 0.3)
% - elevator deflection so that pitching moment=0.

% DATA TAKEN FROM:
%(st): \\vs-fmr-new.mw-ilr.dom.tu-dresden.de\fmr_proj\2_Laufende_Projekte\231206_EndeAR\AVL\SIG_40_cA07T_ST.DAT
%(sb): \\vs-fmr-new.mw-ilr.dom.tu-dresden.de\fmr_proj\2_Laufende_Projekte\231206_EndeAR\AVL\SIG_40_cA07T_SB.DAT
% SABINE's MEASUREMENTS:
% cA_alpha = CLa  ;  cA_alpha = 4.6
% cA_eta   = Cld3 ;  cA_eta   = 0.003
% Cm_alpha = Cma  ;  Cm_alpha = -0.5
% Cm_eta   = Cmd3 ;  Cm_eta   = -0.0077 <-- den nicht nehmen.
% Cm_q     = Cmq  ;  Cm_q     = -10.12

%% -------- Input File for the Aerodynamics of the aircraft ---------------
% Coefficients are calculated using look-up tables
%       -> Either input complete measured data as matrices or input
%          dependencies as formulas/constants
% see P44 or F16 example and check the HOW-TO below


% ----------- Units of the input angles
% change the Units of the input angles, depending on how your data is given
% -> "1" for degree, "0" for rad
% Units of alpha and beta
AeroCoeff.SwitchDegree.alpha_beta        = 0; % "1" for degree, "0" for rad % changed (derivatives which depend on alpha and beta in rad in AVL e.g. CLalpha)
% Units of control angles (elevator,aileron,rudder)
AeroCoeff.SwitchDegree.control_angles    = 1; % "1" for degree, "0" for rad % changed (control surfaces in AVL in degree)


% ----------- Wind-/Stability- to Body-axis transformation
% in case aerodynamic data was obtained in wind or stability axis
% set on and off with 1 & 0 respectively
AeroCoeff.alpha_wind = 1; % for stability- or wind- to body-axis            % alpha data was obtained in stability system
AeroCoeff.beta_wind  = 1; % for stability- to body-axis                     % beta data was obtained in stability system


% ----------- HOW-TO input Aircraft Aerodynamics:
% - first set the input angle range (Grid) at which you have datapoints
%   for "AeroCoeff.XXXXX.Angle = [Angle range];"
% - then input the available data for that grid
%   for "AeroCoeff.XXXXX.Data = [Output data];"
% - in case of analytic linear dependencies set an angle range from [-1,1],
%   for more complicated dependencies (quadratic) a very fine grid is
%   needed. Input your formula for that angle range e.g.: f(angle_range)
% - if certain dependencies are not available, multiply the range with
%   zero or, in case of 2-D-tables with 2 ranges, multiply the varying data
%   with ones


%% -------------------------- Control Coefficients ------------------------
% ---------------------------------- DLDA --------------------------------
AeroCoeff.DLDA.alpha = [-1,1];     
AeroCoeff.DLDA.beta  = [-1,1];

AeroCoeff.DLDA.Data  = [zeros( length(AeroCoeff.DLDA.alpha),length(AeroCoeff.DLDA.beta) )];
                
% ---------------------------------- DLDR --------------------------------
AeroCoeff.DLDR.alpha = [-1,1];     
AeroCoeff.DLDR.beta  = [-1,1];

AeroCoeff.DLDR.Data  = [zeros( length(AeroCoeff.DLDR.alpha),length(AeroCoeff.DLDR.beta) )];
                   
% ---------------------------------- DNDA --------------------------------
AeroCoeff.DNDA.alpha = [-1,1];     
AeroCoeff.DNDA.beta  = [-1,1];                  
                 
AeroCoeff.DNDA.Data  = [zeros( length(AeroCoeff.DNDA.alpha),length(AeroCoeff.DNDA.beta) )];

% ---------------------------------- DNDR --------------------------------
AeroCoeff.DNDR.alpha = [-1,1];     
AeroCoeff.DNDR.beta  = [-1,1];                  
  
AeroCoeff.DNDR.Data  = [zeros( length(AeroCoeff.DNDR.alpha),length(AeroCoeff.DNDR.beta) )];



%% -------------------------- Moment Coefficients -------------------------
% ---------------------------------- C_L ---------------------------------
AeroCoeff.C_L.alpha  = [-1,1];     
AeroCoeff.C_L.beta   = [-1,1];
AeroCoeff.C_L.ail    = [-1,1];
AeroCoeff.C_L.rud    = [-1,1];
             
AeroCoeff.C_L.Data_alphabeta    = [( AeroCoeff.C_L.beta' .* (-0.065542) )... %changed st Clb (old: -0.069921) (positive beta: aircraft nose turns counter clockwise relative to the air if you look at the aircraft from above --> positive beta leads to a negative rolling moment because of the sideforce of the vertical stabilizer which attacks above the cg)
                                  *ones( 1,length(AeroCoeff.C_L.alpha) )]';
                   
AeroCoeff.C_L.Data_ail          = AeroCoeff.C_L.ail.*(-0.005258);            %changed sb Cld2 (Vorzeichen händisch hinzugefügt) (old: -0.005150) (positive ailerondeflection causes negative rolling moment)
AeroCoeff.C_L.Data_rud          = AeroCoeff.C_L.rud.*(0.000034);            %changed sb Cld4 (old: 0.000075) (positive rudder deflection causes positive rolling moment because vertical stabilizer side force attacks above center of gravity) 

% ---------------------------------- C_M ---------------------------------
AeroCoeff.C_M.alpha  = [-10:0.01:45].* pi/180;
AeroCoeff.C_M.eta    = [-14:0.1:14].* pi/180;
AeroCoeff.C_M.beta   = [-15:0.01:15].* pi/180;
AeroCoeff.C_M.alphadot = [-1,0,1];
AeroCoeff.C_M.CT     = [0,1];
AeroCoeff.C_M.Flaps  = [0,10,25,40];
AeroCoeff.C_M.h_alpha = [-1,0,1];
AeroCoeff.C_M.h      = [0,10];

AeroCoeff.C_M.Data_alphaeta   = ( (AeroCoeff.C_M.alpha' .* (-0.5)) * ones(1,length(AeroCoeff.C_M.eta)) )... % changed Sabine Cma (old: -0.980050) (increase in (positive) alpha leads to negative pitching moment (nose down))
                                 + ( ones(length(AeroCoeff.C_M.alpha),1) * (AeroCoeff.C_M.eta .* (-0.0119)) ); % Cmd3 aus "K:\02_Laufende Projekte\231206_EndeAR\AVL\SIG_40_alpha00.DAT" (positive elevator deflection causes negative pitching moment)
                             
AeroCoeff.C_M.Data_sb_alpha   = AeroCoeff.C_M.alpha .* 0;                   % (0 beacause no speedbrakes)
                       
AeroCoeff.C_M.Data_beta       = (AeroCoeff.C_M.beta .^ 2) .* (0);           % (see Tutorial Felix)
AeroCoeff.C_M.Data_CT         = AeroCoeff.C_M.CT .* (0);                    % (no pitching moment due to engine thrust)
                          
AeroCoeff.C_M.Data_0          = [-0.00721, -0.00721, -0.00721, -0.00721];   % Cmtot aus "K:\02_Laufende Projekte\231206_EndeAR\AVL\SIG_40_alpha00.DAT" (Cmtot in AVL with alpha=0 and elevator=0, no flaps so four times the same value)

AeroCoeff.C_M.Data_Gear       = 0;                                          % (no consideration of the gear for the first calculations)

AeroCoeff.C_M.Data_H_alpha    = (AeroCoeff.C_M.h' * AeroCoeff.C_M.h_alpha) .* 0; % (see Tutorail Felix)

AeroCoeff.C_M.Data_alphadot   = AeroCoeff.C_M.alphadot .* 0;                % (see Tutorail Felix)
% Remark:
% currently fed with q instead, because using derivatives would require
% bigger efforts to circumvent an algebraic loop and that's not deemed
% nessecary for now as the error is expected to be quite small

% ---------------------------------- C_N ---------------------------------
AeroCoeff.C_N.alpha  = [-1,0,1];     
AeroCoeff.C_N.beta   = [-5:1:5];
AeroCoeff.C_N.rud    = [-1,0,1];
AeroCoeff.C_N.ail    = [-1,0,1];

AeroCoeff.C_N.Data_alphabeta   = [(AeroCoeff.C_N.beta' .* 0.067843)...      %changed st Cnb (old: 0.090670) (positive beta: aircraft nose turns counter clockwise relative to the air if you look at the aircraft from above --> positive beta leads to a positive yawing moment)
                                   *ones(1,length(AeroCoeff.C_N.alpha) )]';

AeroCoeff.C_N.Data_rud         = [AeroCoeff.C_N.rud .* (-0.001004)];         %changed sb Cnd4 (Vorzeichen händisch hinzugefügt) (old: -0.001240) (positive rudder deflection causes negative yawing moment)
                              
AeroCoeff.C_N.Data_ail         = [AeroCoeff.C_N.ail .* 0.000098];           %changed sb Cnd2 (Vorzeichen händisch entfernt) (old: 0.000258) (one part of the adverse yaw, positive aileron deflection (right flap down) causes positive yawing moment BUT with positive aileron defliction you want to enter a left turn, so the positive yawing moment works against the intended maneuver)
                    
%% --------------------------- Force Coefficients -------------------------
% ---------------------------------- C_X ---------------------------------
AeroCoeff.C_X.alpha   = [-10:0.01:45] .* pi/180;     
AeroCoeff.C_X.eta     = [-15:0.1:15] .* pi/180;
AeroCoeff.C_X.Flaps   = [0,10,25,40];
AeroCoeff.C_X.beta    = [-1,0,1];
AeroCoeff.C_X.CT      = [0,1];
AeroCoeff.C_X.Mach    = [0,1];

% for each flaps setting
AeroCoeff.C_X.Data_alphaeta(:,:,1)     = [ ones(length(AeroCoeff.C_X.eta),1)...            % (Drag over alpha was calculated in xflr5 with constant Liftforce, quadratic approximation was calculated in Polars_Sig_Kadet_LT40.m in the XFLR5-Folder)
                                           *((abs(AeroCoeff.C_X.alpha) .* -0.001981215038*(180/pi))...      % all 4 cases are the same because we are not simulating any flaps for now
                                              +((AeroCoeff.C_X.alpha.^2) .* -0.0004558141065*(180/pi)^2))]';   % (Vorzeichen händisch hinzugefügt) (old:-0.2007*alpha und -0.8862*alpha^2) 
                                                                                           % changed from "\\vs-fmr-new.mw-ilr.dom.tu-dresden.de\fmr_proj\2_Laufende_Projekte\231206_EndeAR\Windkanalversuch\20240123_project-endeAR.pptx" Folie 9 
                                                                                           % values in Folie 9 in rad vs. values in Skript in deg --> *(180/pi) added
AeroCoeff.C_X.Data_alphaeta(:,:,2)     = [ ones(length(AeroCoeff.C_X.eta),1)...
                                           *((abs(AeroCoeff.C_X.alpha) .* -0.001981215038*(180/pi))...
                                              +((AeroCoeff.C_X.alpha.^2) .* -0.0004558141065*(180/pi)^2))]';
AeroCoeff.C_X.Data_alphaeta(:,:,3)     = [ ones(length(AeroCoeff.C_X.eta),1)...
                                           *((abs(AeroCoeff.C_X.alpha) .* -0.001981215038*(180/pi))...
                                              +((AeroCoeff.C_X.alpha.^2) .* -0.0004558141065*(180/pi)^2))]';
AeroCoeff.C_X.Data_alphaeta(:,:,4)     = [ ones(length(AeroCoeff.C_X.eta),1)...
                                           *((abs(AeroCoeff.C_X.alpha) .* -0.001981215038*(180/pi))...
                                              +((AeroCoeff.C_X.alpha.^2) .* -0.0004558141065*(180/pi)^2))]';
                                 
AeroCoeff.C_X.Data_sb_alpha     = AeroCoeff.C_X.alpha .* 0;                 % (no speedbrakes)
                                          
AeroCoeff.C_X.Data_CT_Fl        = AeroCoeff.C_X.CT' * [0, 0, 0, 0];         % (see Tutorial Felix)

AeroCoeff.C_X.Data_beta         = AeroCoeff.C_X.beta .* 0;                  % (see Tutorial Felix)

AeroCoeff.C_X.Data_Mach         = AeroCoeff.C_X.Mach .* 0;                  % (see Tutorial Felix)

AeroCoeff.C_X.Data_0            = [-0.04114337952, -0.04114337952, -0.04114337952, -0.04114337952];     % (Vorzeichen händisch hinzugefügt) (old:-0.0391) 
                                                                                   % changed from "\\vs-fmr-new.mw-ilr.dom.tu-dresden.de\fmr_proj\2_Laufende_Projekte\231206_EndeAR\Windkanalversuch\20240123_project-endeAR.pptx" Folie 9 
                                                                                   % (Drag over alpha was calculated in xflr5 with constant Liftforce, quadratic approximation was calculated in Polars_Sig_Kadet_LT40.m in the XFLR5-Folder, the value here is the constant value of the quadratic formula (at alpha=0), the gear is not inclueded in this value)

AeroCoeff.C_X.Data_Gear         = -0.01;                                    %--->was ist das?? (first guess)
                     
% ---------------------------------- C_Y ---------------------------------
AeroCoeff.C_Y.beta      = [-1,1];
AeroCoeff.C_Y.ail       = [-1,1];
AeroCoeff.C_Y.rud       = [-1,1];

AeroCoeff.C_Y.Data_beta     = AeroCoeff.C_Y.beta .* (-0.238345);            % changed st CYb (old: -0.223856) (positive beta: aircraft nose turns counter clockwise relative to the air if you look at the aircraft from above --> positive beta leads to a neagtive side force because of the vertical stabilizer (and of course the fuselage, but the fuslage was not implemented in AVL))
AeroCoeff.C_Y.Data_ail      = AeroCoeff.C_Y.ail .* (-0.000152);             % changed sb CYd2 (old: -0.001231)
AeroCoeff.C_Y.Data_rud      = AeroCoeff.C_Y.rud .* (0.002412);             % changed sb CYd4 (Vorzeichen händisch entfernt) (old: 0.002651)

AeroCoeff.C_Y0              = 0;                                            % (because of Symmetry over x-z-plane)

% ---------------------------------- C_Z ---------------------------------
AeroCoeff.C_Z.alpha     = [-1,1];     
AeroCoeff.C_Z.beta      = [-1,1];
AeroCoeff.C_Z.Flaps     = [0,10,25,40];
AeroCoeff.C_Z.eta       = [-1,1];
AeroCoeff.C_Z_CT        = [0,1];
AeroCoeff.C_Z.h         = [0,10];

AeroCoeff.C_Z.Data_alphabeta(:,:,1)    = [ ones(length(AeroCoeff.C_Z.beta),1)...
                                           *AeroCoeff.C_Z.alpha .* -4.6]'; % changed Sabine -CLa (old: -4.604505)
AeroCoeff.C_Z.Data_alphabeta(:,:,2)    = AeroCoeff.C_Z.Data_alphabeta(:,:,1);
AeroCoeff.C_Z.Data_alphabeta(:,:,3)    = AeroCoeff.C_Z.Data_alphabeta(:,:,1);
AeroCoeff.C_Z.Data_alphabeta(:,:,4)    = AeroCoeff.C_Z.Data_alphabeta(:,:,1);
                                
AeroCoeff.C_Z.Data_sb_alpha     = AeroCoeff.C_Z.alpha .* 0;                 % (no Speedbrakes)

AeroCoeff.C_Z.Data_eta          = AeroCoeff.C_Z.eta .* -0.008170;           % changed sb CZd3 (old: -0.009192) (positive elevator deflection increases lift of the tail --> increase in overall lift --> z direction of aircraft goes in opposite direction of lift vector)

AeroCoeff.C_Z.Data_CT           = AeroCoeff.C_Z_CT .* 0;                    % (no z-force due to thrust)

AeroCoeff.C_Z.Data_0            = [-0.38539, -0.38539, -0.38539, -0.38539]; % CZtot aus "K:\02_Laufende Projekte\231206_EndeAR\AVL\SIG_40_alpha00.DAT" (AVL alpha=0 and elevator=0)

AeroCoeff.C_Z.Data_H            = AeroCoeff.C_Z.h .* 0;                     % (see Felix Tutorial)


%% ------------------------- Damping Coefficients ------------------------- 
% -------------------------------- C_xq ----------------------------------
% darauf achten die Daten aus body-ais derivatives zu nehmen
AeroCoeff.C_xq.alpha    = [-1,1];

AeroCoeff.C_xq.Data     = AeroCoeff.C_xq.alpha .* 0;                        % (see Felix Tutorial)

% -------------------------------- C_yr ----------------------------------
AeroCoeff.C_yr.alpha    = [-1,1];

AeroCoeff.C_yr.Data     = (AeroCoeff.C_yr.alpha .* 0) + 0.196329;           % changed sb CYr (old: 0.227149) (positive yawing causes positive Sideforce)

% -------------------------------- C_yp ----------------------------------
AeroCoeff.C_yp.alpha    = [-1,1];

AeroCoeff.C_yp.Data     = (AeroCoeff.C_yp.alpha .* 0) + -0.181527;           % changed sb CYp (Vorzeichen händisch hinzugefügt) (old: -0.037524) (positive rolling motion causes neagtive sideforce (maybe because of dihedral))

% -------------------------------- C_zq ----------------------------------
AeroCoeff.C_zq.alpha    = [-1,1];

AeroCoeff.C_zq.Data     = (ones(length(AeroCoeff.C_zq.alpha),1) ) .* (-8.414081);  % changed sb CZq (old: -8.010692) 

% -------------------------------- C_lr ----------------------------------
AeroCoeff.C_lr.alpha    = [-1,1];

AeroCoeff.C_lr.Data     = (AeroCoeff.C_lr.alpha .* 0) + 0.146015;           % changed sb Clr (old: 0.085802) (positive yawing leads to postive rolling moment because the left wing is faster and pruduces mor lift)

% -------------------------------- C_lp ----------------------------------
AeroCoeff.C_lp.alpha    = [-1,1];

AeroCoeff.C_lp.Data     = (AeroCoeff.C_lp.alpha .* 0) + -0.445057;          % changed sb Clp (old: -0.422237) (delf damping of rolling motion)

% -------------------------------- C_mq ----------------------------------
AeroCoeff.C_mq.alpha    = [-1,1];

AeroCoeff.C_mq.Data     = (AeroCoeff.C_mq.alpha .* 0) + -10.12;         % changed Sabine Cmq (old: -10.224503) (self damping of pitching motion)

% -------------------------------- C_nr ----------------------------------
AeroCoeff.C_nr.alpha    = [-1,1];

AeroCoeff.C_nr.Data     = (AeroCoeff.C_nr.alpha .* 0) +  -0.070606;         % changed sb Cnr (old: -0.093360) (self damping of yawing motion)

% -------------------------------- C_np ----------------------------------
AeroCoeff.C_np.alpha    = [-1,1];

AeroCoeff.C_np.Data     = (AeroCoeff.C_np.alpha .* 0) + -0.091560;          % changed sb Cnp (old: -0.019833) (mainfactor for adverse yaw, positive rolling motion leads to neagtive yawing moment)



                   