function [dzita, zita_new, eta_new, q_new] = Integration_d(KinOnly, eta, DH, DH_r, zita, zita_d, tau, PARAM, Ts)
%
% Integration for both kinematic and dynamic simulations, Euler rule
% function  [dzita, zita_new, eta_new, q_new] = Integration(KinOnly, eta, DH, zita, zita_d, tau, PARAM, Ts)
%
% input:
%       KinOnly  dim 1x1     =1 for kinematic simulation
%       eta      dim 6x1     vehicle position/orientation
%       DH/DH_r  dim nx4     DH table
%       zita     dim 6+nx1   system velocity
%       zita_d   dim 6+nx1   desired system velocity 
%       tau      dim 6+nx1   generalized forces 
%       PARAM    struct      parameters for the dynamic simulation
%       Ts       dim 1x1     sampling time
%
% output:
%       dzita    dim 6+nx1   system acceleration 
%       zita_new dim 6+nx1   system velocity at sample i+1 
%       eta_new  dim 6x1     vehicle configuration at sample i+1 
%       q        dim nx1     joint position at sample i+1 
%
% Yuhao Liu   2019/12/30

eta    = CheckVector(eta);
zita   = CheckVector(zita);
zita_d = CheckVector(zita_d);
tau    = CheckVector(tau);

n = size(DH,1);    
q = [DH(:,4);DH_r(:,4)];

if (KinOnly==1)
    dzita = zeros(6+6,1);
    zita_new = zita_d;
else
    dzita = DirectDynamics_d(eta(4:6),DH,DH_r,zita,tau,PARAM); 
    zita_new = zita + Ts*dzita; 
end

Je      = J_e(eta(4:6));
eta_dot = Je\zita(1:6);
q_dot   = zita(7:12);
eta_new = eta + Ts*eta_dot;
q_new   = q + Ts*q_dot;
