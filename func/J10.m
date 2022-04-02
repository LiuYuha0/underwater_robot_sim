function J = J10(n,eta2)
%
% Computes the Jacobian for the task function:
%   vehicle orientation
%
% function  J = J10(n,eta2)
%
% input:
%
%       n     dim 1x1    number of links
%       eta2  dim 3x1    vehicle orientation in rpy
%
% output:
%       J   dim 3x6+n  Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

R_B_I = Rpy2Rot(eta2);
J = [zeros(3,3) R_B_I zeros(3,n)];
