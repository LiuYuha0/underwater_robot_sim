function sigma_tilde = sigma_tilde07(p_d,eta,DH,T_0_B)
%
% Computes the task function:
%   end-effector field-of-view "cameraman" error
%
% function sigma_tilde = sigma_tilde07(p_d,eta,DH,T_0_B)
%
% input:
%       p_d      dim 3x1     point to be framed
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       sigma_tilde    dim 1x1     end-effector field-of-view error
%
% G. Antonelli, Simurv 4.0, 2013

p_d = CheckVector(p_d);
eta = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);
a       = TT(1:3,3);
r       = p_d - eta_ee1;
a_d     = r/norm(r);

sigma_tilde = 0 - sqrt((a_d-a)'*(a_d-a));
