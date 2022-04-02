function [dzita, zita_new, eta_new, q_new] = Integration(KinOnly, eta, DH, zita, zita_d, tau, PARAM, Ts)
%
% Integration for both kinematic and dynamic simulations, Euler rule
% �˶��Ͷ�̬����ļ��ɣ�ŷ������
% function  [dzita, zita_new, eta_new, q_new] = Integration(KinOnly, eta, DH, zita, zita_d, tau, PARAM, Ts)
%
% input:
%       KinOnly  dim 1x1     =1 for kinematic simulation
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table DHϵ
%       zita     dim 6+nx1   system velocity ϵͳ�ٶ�
%       zita_d   dim 6+nx1   desired system velocity ����ϵͳ�ٶ�
%       tau      dim 6+nx1   generalized forces ������
%       PARAM    struct      parameters for the dynamic simulation
%       Ts       dim 1x1     sampling time
%
% output:
%       dzita    dim 6+nx1   system acceleration ϵͳ���ٶ�
%       zita_new dim 6+nx1   system velocity at sample i+1 ��һ���ٶ�
%       eta_new  dim 6x1     vehicle configuration at sample i+1 ��һ��λ��
%       q        dim nx1     joint position at sample i+1 ��һ�̹ؽ�λ��
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
    dzita = DirectDynamics(eta(4:6),DH,zita,tau,PARAM); % ��ǰ��̬��DH���ٶȼ�����һʱ���ٶ�
    zita_new = zita + Ts*dzita; % acc*Ts=vol
end

% ����
Je      = J_e(eta(4:6));
eta_dot = Je\zita(1:6);
q_dot   = zita(7:6+n);
eta_new = eta + Ts*eta_dot;
q_new   = q + Ts*q_dot;
