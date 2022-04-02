function sigma_tilde = sigma_tilde11(psi_d,eta2)
%
% Computes the task function:
%   vehicle yaw error
%
% function sigma_tilde = sigma_tilde11(psi_d,eta2)
%
% input:
%       psi_d         dim 1x1     vehicle desired yaw
%       eta2          dim 3x1     vehicle orientation
%
% output:
%       sigma_tilde   dim 1x1     vehicle yaw error
%
% G. Antonelli, Simurv 4.0, 2013

sigma_tilde = psi_d - eta2(3);