function sigma_tilde = sigma_tilde12(roll_pitch_d,eta2)
%
% Computes the task function:
%   vehicle roll pitch error
%
% function sigma_tilde = sigma_tilde12(roll_pitch_d,eta2)
%
% input:
%       roll_pitch_d  dim 2x1     vehicle desired roll-pitch
%       eta2          dim 3x1     vehicle orientation
%
% output:
%       sigma_tilde   dim 2x1     vehicle roll-pitch error
%
% G. Antonelli, Simurv 4.0, 2013

roll_pitch_d = CheckVector(roll_pitch_d);
eta2         = CheckVector(eta2);

sigma_tilde = roll_pitch_d - eta2(1:2);