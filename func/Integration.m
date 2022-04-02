function [dzita, zita_new, eta_new, q_new] = Integration(KinOnly, eta, DH, zita, zita_d, tau, PARAM, Ts)
%
% Integration for both kinematic and dynamic simulations, Euler rule
% 运动和动态仿真的集成，欧拉法则
% function  [dzita, zita_new, eta_new, q_new] = Integration(KinOnly, eta, DH, zita, zita_d, tau, PARAM, Ts)
%
% input:
%       KinOnly  dim 1x1     =1 for kinematic simulation
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table DH系
%       zita     dim 6+nx1   system velocity 系统速度
%       zita_d   dim 6+nx1   desired system velocity 期望系统速度
%       tau      dim 6+nx1   generalized forces 广义力
%       PARAM    struct      parameters for the dynamic simulation
%       Ts       dim 1x1     sampling time
%
% output:
%       dzita    dim 6+nx1   system acceleration 系统加速度
%       zita_new dim 6+nx1   system velocity at sample i+1 下一刻速度
%       eta_new  dim 6x1     vehicle configuration at sample i+1 下一刻位置
%       q        dim nx1     joint position at sample i+1 下一刻关节位置
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta    = CheckVector(eta);
zita   = CheckVector(zita);
zita_d = CheckVector(zita_d);
tau    = CheckVector(tau);

n = size(DH,1);     % joint number
q = DH(:,4);

if (KinOnly==1)
    dzita = zeros(6+n,1);
    zita_new = zita_d;
else
    dzita = DirectDynamics(eta(4:6),DH,zita,tau,PARAM); % 当前姿态，DH，速度计算下一时刻速度
    zita_new = zita + Ts*dzita; % acc*Ts=vol
end

% 更新
Je      = J_e(eta(4:6));
eta_dot = Je\zita(1:6);
q_dot   = zita(7:6+n);
eta_new = eta + Ts*eta_dot;
q_new   = q + Ts*q_dot;
