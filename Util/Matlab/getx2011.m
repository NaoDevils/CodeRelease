function [ out ] = getx2011(pRef, i, ZMP_err, CoM_err, Acc_err, Gyr_err, params)
%GETX Summary of this function goes here
%   Detailed explanation goes here

format long;

obs=[0;0;0;0];
cur=[0;0;0;0];
filter=[0;0;0;0];
v=0;
struct=writeParam2011(params);
out.struct=struct;
cont=0;


    
for k=1:i,
    Ym(1,k)=CoM_err(k)+obs(1);
    Ym(2,k)=Gyr_err(k)+obs(2);
    Ym(3,k)=Acc_err(k)+obs(3);
    Ym(4,k)=ZMP_err(k)+obs(4); % oder pRef(k)?
    
 
    u = getu(struct.Gd, pRef, struct.N, k-1);   
    cont =  (-struct.Gi * v - struct.Gx * obs - u);   
    obs = struct.A0 * obs + struct.L * (Ym(:,k)-obs) + struct.b0 * cont;
    v = v + struct.c0 * obs - pRef(k);   
    out.v(k)=v;
    out.obs(k, :)=obs(:);
end

t2=1:i-2;
dCoM=diff(out.obs(:, 1)*100);
ddCoM=diff(dCoM)*100;
calc_zmp=out.obs(1:size(ddCoM), 1)-(struct.z_h/struct.g)*ddCoM;

out.ddCoM=ddCoM;

for k=1:i-2
   zmp(k)=out.obs(k, 1)-calc_zmp(k);
end


for k=1:i-2
    ZMP_err(k)=ZMP_err(k)+out.obs(k, 4);
end

zmp_acc=0;
speedx=diff(out.obs(:,1))*50;
speedx(size(out.obs, 1)-1)=speedx(size(out.obs, 1)-2);
speedx(size(out.obs, 1))=speedx(size(out.obs, 1)-1);
for t=1:size(out.obs, 1)
   zsp(t)= out.obs(t,1)+0.153*speedx(t); %0.153
   zmp_acc=zmp_acc+(-pRef(t)+out.obs(t, 1));
   zsp2(t)= zmp_acc*0.02;
end

t=1:i;

% Simple Plots

%ZSP Plots
%plot(t, pRef(t), t, out.cur(t,1), t, out.cur(t,3));
%legend('pRef', 'cur CoM', 'cur ZMP')
% h=plot(t, pRef(t), t, out.cur(t,1), t, out.cur(t,3), t, zsp(t), t, zsp2(t));
% legend('pRef', 'cur CoM', 'cur ZMP', 'ZSP', 'ZSP2');
% set(h,{'LineStyle'}, {'-';'-';'-';'-';'-.'});

%Plots for testing
% figure('Position',[ 0, 0, 1000, 1000]);
% h=plot(t, pRef(t), t, out.obs(t,1), t, out.obs(t,3), t2, ZMP_err(t2), t, out.obs(t, 2));
% legend('pRef', 'obs CoM', 'obs ZMP', 'calculated ZMP', 'observed Speed');
% set(h,{'LineWidth'},{2;1;2;1;1});


% Plots for new version
h=plot(t, ZMP_err(t), t, pRef(t), t, out.obs(t,1), t, out.obs(t,3)/50, t, out.obs(t,4), t, out.v(t), t2, calc_zmp(t2), t2, ddCoM(t2)/50);
set(h,{'LineWidth'},{2;2;1;1;1;1;2;1});
set(h,{'LineStyle'}, {'--';'.';'-.';'--';'--';'-';'-';'-'});
axis([0,400,-0.1,.1])
legend('ZMP err', 'pRef', 'obs CoM', 'obs Acc 1/50', 'obs ZMP', 'cur Err', 'calculated ZMP', 'Acc 1/50');

% Plots for paper with sensor
% t=0:0.02:(i-1)/50
% t2=1:i
% figure('Position',[ 0, 0, 1000, 450]);
% h = plot(t, ZMP_err(t2), t, pRef(t2), t, out.cur(t2,1), t, out.cur(t2,3));
% set(h,{'LineStyle'}, {'-';'-';'-.';'--'});
% set(h,{'LineWidth'},{2;1;1;1});
% set(h,{'Color'},{'k';'k';'k';'k'})
% legend('ZMP err', 'pRef', 'cur CoM', 'cur ZMP');
% xlabel('time [s]');
% ylabel('y [m]');
% axis([0; 4; -0.05; 0.05]);
% legend('Location', 'NorthWest', 'measured ZMP','reference ZMP','CoM','calculated ZMP');
% exportfig(gcf, 'zmp_err.eps', 'bounds', 'tight', 'Format', 'eps');
% system('epstopdf zmp_err.eps');

% Plots for paper without sensor
% t=0:0.02:(i-1)/50;
% t2=1:i;
% figure('Position',[ 0, 0, 300, 300]);
% h = plot(t, pRef(t2), t, out.cur(t2,1), t, out.cur(t2,3));
% set(h,{'LineStyle'}, {'-';'-.';'-'});
% set(h,{'LineWidth'},{2;1;1});
% set(h,{'Color'},{'k';'k';'k'});
% xlabel('time [s]');
% ylabel('y [m]');
% axis([1.5; 2.5; -0.04; 0.04]);
% legend(h, 'Location', 'SouthEast', 'ref ZMP','CoM','calc ZMP');
% out.h=h;
% exportfig(gcf, 'N.eps', 'bounds', 'tight', 'Format', 'eps');
% system('epstopdf N.eps') 

% Plots for paper
% t=0:0.02:(i-1)/50;
% t2=1:i;
% figure('Position',[ 0, 0, 500, 300]);
% h = plot(t, pRef(t2), t, out.cur(t2,1));
% set(h,{'LineStyle'}, {'-';'-.'});
% set(h,{'LineWidth'},{2;1});
% set(h,{'Color'},{'k';'k'});
% xlabel('time [s]');
% ylabel('y [m]');
% axis([0; 2; -0.0005; 0.0105]);
% legend(h, 'Location', 'SouthEast', 'ref ZMP','CoM');
% out.h=h;
% exportfig(gcf, 'RefCoM.eps', 'bounds', 'tight', 'Format', 'eps');
% system('epstopdf RefCoM.eps')



