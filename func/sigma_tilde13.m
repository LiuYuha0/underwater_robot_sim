function sigma_tilde = sigma_tilde13(eta_1d,eta)
%
% Computes the task function:
%   vehicle error position norm
%
% function sigma_tilde = sigma_tilde13(eta_1d,eta,DH)
%
% input:
%       eta_1d   dim 3x1     vehicle desired position
%       eta      dim 6x1     vehicle position/orientation
%
% output:
%       sigma_tilde    dim 1x1    end-effector error position norm
%
% G. Antonelli, Simurv 4.0, 2013

eta_1d = CheckVector(eta_1d);
eta    = CheckVector(eta);

eta_1 = eta(1:3);
sigma_tilde = 0 - sqrt((eta_1d-eta_1)'*(eta_1d-eta_1));
