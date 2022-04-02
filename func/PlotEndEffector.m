function PlotEndEffector(t,eta_ee_1d,eta_ee1,eta_ee_quat_d,eta_ee_quat)
%
% PlotEndEffector(t,eta_ee_1d,eta_ee1,eta_ee_quat_d,eta_ee_quat)
%
% input:
%       t              dim nptix1      time vector
%       eta_ee_1d      dim 3xnpti      desired ee position
%       eta_ee1        dim 3xnpti      ee position
%       eta_ee_quat_d  dim 4xnpti      desired ee quaternion
%       eta_ee_quat    dim 4xnpti      ee quaternion
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

cla

subplot(211)
plot(t,eta_ee_1d,'b-.',t,eta_ee1,'r')
title('ee position')
xlabel('t [s]'),ylabel('ee [m]')
grid

subplot(212)
plot(t,eta_ee_quat_d(1:3,:),'b')
hold on
plot(t,eta_ee_quat_d(4,:),'b--','LineWidth',2)
plot(t,eta_ee_quat(1:3,:),'r')
plot(t,eta_ee_quat(4,:),'r--','LineWidth',2)
title('ee quaternion (blu-desired, red-measured)')
xlabel('t [s]'),ylabel('ee [-]')
grid

