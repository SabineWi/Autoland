function G = BuildGeneralizedPlant(Plant, Weights)
%GETGENERALIZEDPLANT Summary of this function goes here
%   Detailed explanation goes here

% init variables for improved readability of code 
Ve = Weights.Ve; 
Vu = Weights.Vu; 
Vd = Weights.Vd; 

We = Weights.We; 
Wu = Weights.Wu; 

P = Plant; 

% define weights 
W1 = We*inv(Ve);
W2 = Wu*inv(Vu);

% dimensions
sizeR = length(Ve); 
sizeD = length(Vd); 
sizeU = length(Vu); 

% setup sysic
systemnames = 'P W1 W2 Ve Vd'; 
inputvar = strcat('[r{',num2str(sizeR),'}; d{',num2str(sizeD),'}; u{',num2str(sizeU),'}]');
outputvar = strcat('[W1; W2; Ve-P(1:',num2str(sizeR),')]');
input_to_P = '[u + Vd]'; 
input_to_W1 = strcat('[Ve - P(1:',num2str(sizeR),')]');
input_to_W2 = '[u]'; 
input_to_Vd = '[d]'; 
input_to_Ve = '[r]';
            
G = sysic; 


end

