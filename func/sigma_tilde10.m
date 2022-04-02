function sigma_tilde = sigma_tilde10(eta2_quat_d,eta2)
%
% Computes the task function:
%   vehicle orientation error
%
% function sigma_tilde = sigma_tilde10(eta2_quat_d,eta2)
%
% input:
%       eta2_quat_d   dim 4x1     vehicle desired orientation
%                                 expressed in quaternions
%       eta2          dim 3x1     vehicle orientation in rpy
%
% output:
%       sigma_tilde   dim 3x1     vehicle orientation quaternion error
%
% G. Antonelli, Simurv 4.0, 2013

eta2_quat_d = CheckVector(eta2_quat_d);
eta2        = CheckVector(eta2);

% quaternion associated to R_B_I (from body-fixed to inertial)
e = Rpy2Quat(eta2);

sigma_tilde = e(4)*eta2_quat_d(1:3) - eta2_quat_d(4)*e(1:3) +...
             cross(e(1:3),eta2_quat_d(1:3));
         