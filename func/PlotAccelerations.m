function PlotAccelerations(t,dzita)
%
% PlotVelocities(t,dzita)
%
% input:
%       t     dim npti x 1      time vector
%       dzita dim 6+n  x npti   system accelerations
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

n = size(dzita,1)-6;
cla

subplot(311)
plot(t,dzita(1:3,:))
title('vehicle linear acc')
xlabel('t [s]'),ylabel('dnu1 [m/s2]')
grid

subplot(312)
plot(t,dzita(4:6,:)*180/pi)
title('vehicle angular acc')
xlabel('t [s]'),ylabel('dnu2 [deg/s2]')
grid

subplot(313)
plot(t,dzita(6+1:6+n,:)*180/pi)
title('joint acc')
xlabel('t [s]'),ylabel('ddq [deg/s2]')
