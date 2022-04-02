function sigma_tilde = sigma_tilde09(q,qd)
%
% Computes the task function:
%   robot nominal position error
%   �����˱��λ�����
% function sigma_tilde = sigma_tilde08(q,qd,i)
%
% input:
%       q   dim nx1    joint positions �ؽڽǶ�
%       qd  dim nx1    desired joint position �����ؽڽǶ�
%
% output:
%       sigma_tilde   dim nx1  single joint position error ���ؽڽ����
%
% G. Antonelli, Simurv 4.0, 2013

q  = CheckVector(q);
qd = CheckVector(qd);

sigma_tilde = qd - q;