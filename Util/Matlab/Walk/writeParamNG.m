function [ ret ] = writeParamNG(params)
%WRITEPARAM Summary of this function goes here
%   Detailed explanation goes here
path=params.path;
format long;

g = params.g;		
z_h = params.z_h;
dt = params.dt;		

Qx = params.Qx;
Qe = params.Qe;
Ql = params.Ql;

R = params.R;
RO = params.RO;
N = params.N;


A0 = [1, dt, 0; g/z_h*dt, 1, -g/z_h*dt; 0, 0, 1];
b0 = [0; 0; dt];
c0 = [0, 0, 1];

Co=ctrb(A0, b0);
unco=length(A0)-rank(Co)

Bt(1,1)=c0*b0;
Bt(2:4,1)=b0(1:3);
It(1,1)=1;
It(2:4,1)=0;
Ft(1,1:3)=c0*A0;
Ft(2:4,1:3)=A0(1:3,1:3);
Qt(1:4, 1:4)=0;
Qt(1,1)=Qe;
Qt(2:4,2:4)=c0'*Qx*c0;
At(1:4,1)=It;
At(1:4,2:4)=Ft;

Co=ctrb(At, Bt);
unco=length(At)-rank(Co)



%[Pt, L, G, report]=dare(At, Bt, Qt, R);
Pt=dare(At, Bt, Qt, R);
%report

Gx = (R+Bt'*Pt*Bt)^-1 * Bt'*Pt*Ft;
Gi = (R+Bt'*Pt*Bt)^-1 * Bt'*Pt*It;

Ac = At - Bt*(R + Bt'*Pt*Bt)^-1 * Bt'*Pt*At
X = -Ac'*Pt*It;
Gd(1) = -Gi;
for i=2:N,
    Gd(i) = (R + Bt'*Pt*Bt)^-1*Bt'*X;
    X = Ac'*X;
end

Cm=[1,0,0;
    0,0,1];

Ob=obsv(A0, Cm);
ObserverRank=rank(Ob)
L = dlqr(A0.', Cm.', Ql , RO)';


ret.A0=A0;
ret.L=L;
ret.b0=b0;
ret.N=N;
ret.b0=b0;
ret.c0=c0;
ret.z_h=z_h;
ret.g=g;
ret.Gi=Gi;
ret.Gx=Gx;
ret.Gd=Gd;
ret.It=It;
ret.Ac=Ac;
ret.Bt=Bt;
ret.Cm=Cm;

if isempty(path)
    return;
end
    
fid = fopen(path,'w');

% ---------------- z_h ---------------

fprintf(fid,'%-190.30e\r\n',z_h);

% ---------------- dt ----------------

fprintf(fid,'%-190.30e\r\n',dt);

% ---------------- N  ----------------

fprintf(fid,'%-190.30e\r\n',N);

% ---------------- L  ----------------

fprintf(fid,'%-190.30e\r\n',L(1,1));
fprintf(fid,'%-190.30e\r\n',L(2,1));
fprintf(fid,'%-190.30e\r\n',L(3,1));

fprintf(fid,'%-190.30e\r\n',L(1,2));
fprintf(fid,'%-190.30e\r\n',L(2,2));
fprintf(fid,'%-190.30e\r\n',L(3,2));

% ---------------- A0 ----------------

fprintf(fid,'%-190.30e\r\n',A0(1,1));
fprintf(fid,'%-190.30e\r\n',A0(1,2));
fprintf(fid,'%-190.30e\r\n',A0(1,3));

fprintf(fid,'%-190.30e\r\n',A0(2,1));
fprintf(fid,'%-190.30e\r\n',A0(2,2));
fprintf(fid,'%-190.30e\r\n',A0(2,3));

fprintf(fid,'%-190.30e\r\n',A0(3,1));
fprintf(fid,'%-190.30e\r\n',A0(3,2));
fprintf(fid,'%-190.30e\r\n',A0(3,3));

% ---------------- Gi ----------------

fprintf(fid,'%-190.30e\r\n', Gi);

% ---------------- Gx ----------------

fprintf(fid,'%-190.30e\r\n',Gx(1));
fprintf(fid,'%-190.30e\r\n',Gx(2));
fprintf(fid,'%-190.30e\r\n',Gx(3));

% ---------------- b0 ----------------

fprintf(fid,'%-190.30e\r\n',b0(1));
fprintf(fid,'%-190.30e\r\n',b0(2));
fprintf(fid,'%-190.30e\r\n',b0(3));

% ---------------- c0 ----------------

fprintf(fid,'%-190.30e\r\n',c0(1));
fprintf(fid,'%-190.30e\r\n',c0(2));
fprintf(fid,'%-190.30e\r\n',c0(3));

% ---------------- Gd ----------------

for i=1:N,
    fprintf(fid,'%-190.30e\r\n',Gd(i));
end

status = fclose(fid);


