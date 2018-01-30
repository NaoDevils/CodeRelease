function [ params ] = Nao2011(name)

params.g = 9.8;		
params.z_h = 0.26;           % [m]   H�he, in der der Schwerpunkt gef�hrt werden soll.
                                
params.dt = 0.01;		
params.R = 1*10^-10;
params.N = 100;
params.Qx = 1;
params.Qe = 1; 


params.Ql = [ 1, 0, 0, 0 ;
                         0, 1, 0, 0 ;
                         0, 0, 1, 0 ;
                         0, 0, 0, 1 ];
         
params.RO = [10,    0,   0,    0 ;
                            0,    1,   0,    0 ; 
                            0,    0,  10^4,    0 ; 
                            0,    0,   0,  10];
if isempty(name)
    params.path='';
else
    params.path=['../../Config/Robots/', name, '/ZMPIPController.dat'];
end
