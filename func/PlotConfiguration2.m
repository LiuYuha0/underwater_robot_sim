function PlotConfiguration2(t,eta,q,eta_d,q_d)
%
% PlotConfiguration2(t,eta,q,eta_d,q_d)
%
% input:
%       t   dim nptix1      time vector
%       eta dim 6xnpti      vehicle configuration
%       q   dim nxnpti      joint position
%       eta dim 6xnpti      desired vehicle configuration
%       q   dim nxnpti      desired joint position
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

cla

subplot(311)
plot(t,eta(1:3,:))
hold on
plot(t,eta_d(1:3,:),'--')
title('vehicle position (solid-real, dashed-desired)')
xlabel('t [s]'),ylabel('eta1 [m]')
grid

subplot(312)
plot(t,eta(4:6,:)*180/pi)
hold on
plot(t,eta_d(4:6,:)*180/pi,'--')
title('vehicle orientation rpy (solid-real, dashed-desired)')
xlabel('t [s]'),ylabel('eta2 [deg]')
grid

subplot(313)
plot(t,q*180/pi)
hold on
plot(t,q_d*180/pi,'--')
title('joint position (solid-real, dashed-desired)')
xlabel('t [s]'),ylabel('q [deg]')
grid
