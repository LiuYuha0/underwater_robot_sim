function J = J09(q)
%
% Computes the Jacobian for the task function:
%   robot nominal positin
%
% function  J = J09(q)
%
% input:
%
%       q   dim nx1    joint positions
%
% output:
%       J   dim nx6+n  Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

q = CheckVector(q);

n = size(q,1);

J = [zeros(n,6) eye(n,n)];
