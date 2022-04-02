function J_ko = J_ko_rpy(rpy)
%
% Jacobian to transform derivative of the
%	Euler angles to body-fixed angular velocity
%
% function J_ko = J_ko_rpy(rpy)
%
% input:
%       rpy     dim 3x1     roll-pitch-yaw angles
%
% output:
%       J_ko    dim 3x3     Jacobian matrix
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

phi   = rpy(1);
theta = rpy(2);
%psi   = rpy(3);

%cp = cos(psi);		sp = sin(psi);
ct = cos(theta);	st = sin(theta);
cf = cos(phi);		sf = sin(phi);

J_ko = [1   0 -st;
			  0  cf  ct*sf;
			  0 -sf  ct*cf];
