function [ params ] = NaoV3(name)

params.g = 9.8;		
params.z_h = 0.265;           % [m]   Höhe, in der der Schwerpunkt geführt werden soll.
                                % 0.24
params.dt = 0.01;		
params.R = 1*10^-10;
params.N = 100;
params.Qx = 0.4;  % 0.04 0.3 0.25 0.4 0.1 0.1
params.Qe = 0.3;   % 0.4 0.4 0.2 0.3 0.1 1.0
% Verbesserter Lauf: Qx = 0.8, Qe = 0.4

params.Ql = [1,0,0;0,1,0;0,0,1];
params.RO = 10; %geht mit 10


params.path=['../../Config/Robots/', name, '/ZMPIPController.dat'];

% Parameter für V3 und 1-2.5s:
% 
% params.g = 9.8;		
% params.z_h = 0.245;         
% params.dt = 0.02;		
% params.R = 1*10^-10;
% params.N = 50;
% params.Qx = 0.4; 
% params.Qe = 0.3;
% params.Ql = [1,0,0;0,1,0;0,0,1];
% params.RO = 10^-10; 

% neuste Parameter für 4s:
% 
% params.dt = 0.02;		
% params.R = 1*10^-10;
% params.N = 60;
% params.Qx = 0.1;  
% params.Qe = 0.8;
% params.Ql = [0,0,0;0,0,0;0,0,1];
% params.RO = 15;


% Parameter für 4s:
% 
% params.Qx = 0.1;
% params.Qe = 1.0;
% params.Ql = [0,0,0;0,0,0;0,0,1];
% params.RO = 15;

% Parameter für 1s:
% 
% params.dt = 0.02;		
% params.R = 1*10^-10;
% params.N = 60;
% params.Qx = 0.25;
% params.Qe = 0.2; 

% Parameter für Modellfehler-Test:
% 
% params.g = 9.8;		
% params.z_h = 0.230;         
% params.dt = 0.02;		
% params.R = 1*10^-10;
% params.N = 60;
% params.Qx = 1; 
% params.Qe = 1; 
% params.Ql = [1,0,0;0,1,0;0,0,1];
% params.RO = 10^-3;

% Parameter für Nao V3:
% 
% params.g = 9.8;		
% params.z_h = 0.240;          
%                                
% params.dt = 0.02;		
% params.R = 1*10^-10;
% params.N = 60;
% params.Qx = 0.4;  
% params.Qe = 0.3;   
% 
% 
% params.Ql = [0,0,0;0,0,0;0,0,1];
% params.RO = 20;
% 
% params.path=['../../Config/Robots/', name, '/ZMPIPController.dat'];

