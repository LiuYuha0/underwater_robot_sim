function J = J08(q,i)
%
% Computes the Jacobian for the task function:
%   single joint position
%
% function  J = J08(q,i)
%
% input:
%
%       q   dim nx1    joint positions
%       i   dim 1x1    joint to be controlled
%
% output:
%       J   dim 1x6+n  Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

q = CheckVector(q);

n = size(q,1);

J = zeros(1,6+n);
J(1,6+i) = 1;
