function [ params ] = NaoNG(name)

params.g = 9.8;		
params.z_h = 0.26;           % [m]   Höhe, in der der Schwerpunkt geführt werden soll.
                                % 0.24
params.dt = 0.01;		
params.R = 1*10^-10;
params.N = 100;
params.Qx = 1;  % 0.04 0.3 0.25 0.4 0.1 0.1
params.Qe = 1;   % 0.4 0.4 0.2 0.3 0.1 1.0
% Verbesserter Lauf: Qx = 0.8, Qe = 0.4

params.Ql = [ 1, 0, 0 ;
              0, 1, 0 ;
              0, 0, 1 ];
         
params.RO = [10,  0      ;
             0,  10]; 

if isempty(name)
    params.path='';
else
    params.path=['../../Config/Robots/', name, '/ZMPIPControllerNG.dat'];
end
