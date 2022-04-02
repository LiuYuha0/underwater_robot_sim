function J = J11(eta2,n)
%
% Computes the Jacobian for the task function:
%   vehicle yaw
%
% function  J = J11(n)
%
% input:
%
%       eta2  dim 3x1    vehicle orientation
%       n     dim 1x1    number of links
%
% output:
%       J     dim 1x6+n  Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta2        = CheckVector(eta2);

J_ko = J_ko_rpy(eta2);

J = [zeros(1,3) [0 0 1]*inv(J_ko) zeros(1,n)];
