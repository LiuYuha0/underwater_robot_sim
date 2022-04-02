function sigma_tilde = sigma_tilde08(q,qd,i)
%
% Computes the task function:
%   single joint position error
%
% function sigma_tilde = sigma_tilde08(q,qd,i)
%
% input:
%       q   dim nx1    joint positions
%       qd  dim 1x1    desired joint position
%       i   dim 1x1    joint to be controlled
%
% output:
%       sigma_tilde   dim 1x1  single joint position error
%
% G. Antonelli, Simurv 4.0, 2013

q = CheckVector(q);

sigma_tilde = qd - q(i);