function sigma_tilde = sigma_tilde09(q,qd)
%
% Computes the task function:
%   robot nominal position error
%   机器人标称位置误差
% function sigma_tilde = sigma_tilde08(q,qd,i)
%
% input:
%       q   dim nx1    joint positions 关节角度
%       qd  dim nx1    desired joint position 期望关节角度
%
% output:
%       sigma_tilde   dim nx1  single joint position error 单关节角误差
%
% G. Antonelli, Simurv 4.0, 2013

q  = CheckVector(q);
qd = CheckVector(qd);

sigma_tilde = qd - q;