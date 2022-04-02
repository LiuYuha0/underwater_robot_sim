function PlotConfiguration(t,eta,q)
%
% PlotConfiguration(t,eta,q)
%
% input:
%       t   dim nptix1      time vector
%       eta dim 6xnpti      vehicle configuration
%       q   dim 6xnpti      joint position
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

cla


subplot(311)
plot(t,eta(1:3,:))
title('vehicle position')
xlabel('t [s]'),ylabel('eta1 [m]')
grid

subplot(312)
plot(t,eta(4:6,:)*180/pi)
title('vehicle orientation rpy')
xlabel('t [s]'),ylabel('eta2 [deg]')
grid

subplot(313)
plot(t,q*180/pi)
title('joint position')
xlabel('t [s]'),ylabel('q [deg]')
