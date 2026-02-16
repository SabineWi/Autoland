%% Prepare a Flight Gear Visualisation and Joystick input

% Attention:
% - For Joystick input the 3D-Animation Add-On is required
% - Currently the runfg.bat file may need some manual editing to run properly

% Uncomment the necessary Simulink blocks
set_param('Flight_Simulation/FlightGear_Interface/FG_simpleInterface','commented','off');
set_param('Flight_Simulation/Joystick_Input','commented','off');


%% Get the IP adress of the computer to connect flight gear correctly

IPaddress.address = java.net.InetAddress.getLocalHost;
IPaddress.ip = char(IPaddress.address.getHostAddress);

IPaddress.idx_dot = find(ismember(IPaddress.ip, '.'));

% IP address has the form M.N.P.Q
IPaddress.ipM = str2double(IPaddress.ip(1:IPaddress.idx_dot(1)-1));
IPaddress.ipN = str2double(IPaddress.ip(IPaddress.idx_dot(1)+1 : IPaddress.idx_dot(2)-1));
IPaddress.ipP = str2double(IPaddress.ip(IPaddress.idx_dot(2)+1 : IPaddress.idx_dot(3)-1));
IPaddress.ipQ = str2double(IPaddress.ip(IPaddress.idx_dot(3)+1 : end));

% Set IP-address automatically in Simulink blocks for FG
set_param('Flight_Simulation/FlightGear_Interface/FG_simpleInterface','DestinationIpAddress',IPaddress.ip);
set_param('Flight_Simulation/FlightGear_Interface/GenerateRunScript','LocalAddress',IPaddress.ip);
