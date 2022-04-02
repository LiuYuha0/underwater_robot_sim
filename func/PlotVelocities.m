function PlotVelocities(t,zita)
%
% PlotVelocities(t,zita)
%
% input:
%       t    dim npti x 1      time vector
%       zita dim 6+n  x npti   system velocities
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

n = size(zita,1)-6;
cla

subplot(311)
plot(t,zita(1:3,:))
title('vehicle linear velocities')
xlabel('t [s]'),ylabel('nu1 [m/s]')
grid

subplot(312)
plot(t,zita(4:6,:)*180/pi)
title('vehicle angular velocities')
xlabel('t [s]'),ylabel('nu2 [deg/s]')
grid

subplot(313)
plot(t,zita(6+1:6+n,:)*180/pi)
title('joint velocities')
xlabel('t [s]'),ylabel('dq [deg/s]')
