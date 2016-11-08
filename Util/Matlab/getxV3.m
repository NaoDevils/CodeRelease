function [ out ] = getxV3(pRef, i, ZMP_err, params)
%GETX The ZMP/IP-Controller.
%   pRef:       The reference ZMP.
%   i:          End frame.
%   ZMP_err:    The difference between the desired and measure ZMP.
%   params:     Parameters for the controller.
%  Example call: getxV3(zmp_ref, 400, zmp_err, NaoV3(''))

format long;

obs=[0;0;0];
cur=[0;0;0];
filter=[0;0;0];
v=0;
struct=writeParamV3(params);
out.struct=struct;
cont=0;
for k=5:i,
%     ZMP_err(k)=ZMP_err(k)+obs(3);
%     
%     u = getu(struct.Gd, pRef, struct.N, k-1);   
%     cont =  (-struct.Gi * v - struct.Gx * obs - u); 
%     obs = struct.A0 * obs + struct.L * (-obs(3)+ZMP_err(k)) + struct.b0 * cont;
%     cur = struct.A0 * obs + struct.b0 * cont;
%     
%     v = v + struct.c0 * obs - pRef(k);   

    ZMP_err(k)=ZMP_err(k)+obs(3);
    u = getu(struct.Gd, pRef, struct.N, k-1);   
    cont =  (-struct.Gi * v - struct.Gx * obs - u);
    obs = struct.A0 * obs + struct.L * (-obs(3)+ZMP_err(k)) + struct.b0 * cont;
    v = v + struct.c0 * obs - pRef(k); 
    
    out.cur(k, :)=obs(:);
    out.v(k)=v;
    out.obs(k, :)=obs(:);
end

t2=1:i-2;
dCoM=diff(out.cur(:, 1)*100);
ddCoM=(diff(dCoM)*100);
calczmp=(struct.z_h/struct.g)*ddCoM;
out.ddCoM=ddCoM;
%ZMP_err(1:i-3)=ZMP_err(1:i-3)+calczmp(1:i-3)';
for k=1:i-2
   zmp(k)=out.cur(k, 1)-calczmp(k);
end

t=1:i;

% Simple Plots
% plot(t, pRef(t), t, out.cur(t,1), t, out.cur(t,3));
% legend('pRef', 'cur CoM', 'cur ZMP');

% Plots for old version
% h=plot(t, ZMP_err(t), t, pRef(t), t, out.cur(t,1), t, out.cur(t,3),  t, out.obs(t,1), t, out.obs(t,3), t, out.v(t), t2, zmp(t2), t2, ddCoM(t2)/10);
% set(h,{'LineWidth'},{2;2;2;2;1;1;1;1;1});
% set(h,{'LineStyle'}, {'--';'.';'-.';'--';'-';'-';'-';'-';'-'});
% axis([0,200,-0.1,.1])
% legend('ZMP err', 'pRef', 'cur CoM', 'cur ZMP', 'obs CoM', 'obs ZMP', 'cur Err', 'calculated ZMP', 'Acc 1/10');


% Plots for new version
h=plot(t, ZMP_err(t), t, pRef(t), t, out.obs(t,1), t, out.obs(t,3), t, out.v(t), t2, zmp(t2), t2, ddCoM(t2)/10);
set(h,{'LineWidth'},{2;2;1;1;1;2;1});
set(h,{'LineStyle'}, {'--';'.';'-.';'--';'-';'-';'-'});
axis([0,400,-0.1,.1])
legend('ZMP err', 'pRef', 'obs CoM', 'obs ZMP', 'cur Err', 'calculated ZMP', 'Acc 1/10');

