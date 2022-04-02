function sigma_tilde = sigma_tilde14(p_obst,d,eta)
%
% Computes the task function:
%   vehicle error position norm
%
% function sigma_tilde = sigma_tilde14(p_obst,d,eta)
%
% input:
%       p_obst   dim 3x1     vehicle desired position
%       d        dim 1x1     desired distance from obstacle
%       eta      dim 6x1     vehicle position/orientation
%
% output:
%       sigma_tilde    dim 1x1    end-effector error position norm
%
% G. Antonelli, Simurv 4.0, 2013

p_obst = CheckVector(p_obst);
eta    = CheckVector(eta);

eta_1 = eta(1:3);
sigma_tilde = d - sqrt((p_obst-eta_1)'*(p_obst-eta_1));
