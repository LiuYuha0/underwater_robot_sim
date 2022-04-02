function PlotForces(t,tau)
%
% PlotVelocities(t,tau)
%
% input:
%       t    dim npti x 1      time vector
%       tau  dim 6+n  x npti   system generalized forces
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

n = size(tau,1)-6;
cla

subplot(311)
plot(t,tau(1:3,:))
title('vehicle linear forces')
xlabel('t [s]'),ylabel('tau1 [N]')
grid

subplot(312)
plot(t,tau(4:6,:))
title('vehicle moments')
xlabel('t [s]'),ylabel('tau2 [Nm]')
grid

subplot(313)
plot(t,tau(6+1:6+n,:))
title('joint torques')
xlabel('t [s]'),ylabel('tauq [Nm]')
grid
