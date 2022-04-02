function J = J_e(rpy)
%
% Jacobian to transform derivative of the 
% vehicle position + Euler angles
% to body-fixed linear and angular velocities
%
% function J = J_e(rpy)
%
% input:
%       rpy     dim 3x1     roll-pitch-yaw angles
%
% output:
%       J    dim 6x6     Jacobian matrix
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

R = Rpy2Rot(rpy);

J = [R'          zeros(3,3);
		 zeros(3,3)  J_ko_rpy(rpy)];
