function J = J12(eta2,n)
%
% Computes the Jacobian for the task function:
%   vehicle roll-pitch
%
% function  J = J12(n)
%
% input:
%
%       eta2  dim 3x1    vehicle orientation
%       n     dim 1x1    number of links
%
% output:
%       J     dim 2x6+n  Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta2 = CheckVector(eta2);

J_ko = J_ko_rpy(eta2);

J = [zeros(2,3) [1 0 0;0 1 0]*inv(J_ko) zeros(2,n)];
