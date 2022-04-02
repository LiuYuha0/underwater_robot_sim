function sigma_tilde = sigma_tilde04(eta_ee_quat_d,eta,DH,T_0_B)
%
% Computes the task function:
%   end-effector orientation error
%
% function sigma_tilde = sigma_tilde04(eta_ee_quat_d,eta,DH,T_0_B)
%
% input:
%       eta_ee_quat_d dim 4x1     end-effector desired orientation
%                                 expressed in quaternions
%       eta           dim 6x1     vehicle position/orientation
%       DH            dim nx4     Denavit-Hartenberg table
%       T_0_B         dim 4x4     Homogeneous transformation 
%                                 matrix from vehicle to zero frame
%
% output:
%       sigma_tilde         dim 3x1     end-effector orientation quaternion error
%
% G. Antonelli, Simurv 4.0, 2013

eta_ee_quat_d = CheckVector(eta_ee_quat_d);
eta           = CheckVector(eta);

% quaternion associated to R_ee_I (from end-effector to inertial)
TT = DirectKinematics(eta,DH,T_0_B);
eta_ee_quat = Rot2Quat(TT(1:3,1:3));

angle = atan2(norm(cross(eta_ee_quat_d(1:3),eta_ee_quat(1:3))),dot(eta_ee_quat_d(1:3),eta_ee_quat(1:3)));
if (angle>(pi/2))
    eta_ee_quat_d = - eta_ee_quat_d;
end

sigma_tilde = eta_ee_quat(4)*eta_ee_quat_d(1:3) -...
             eta_ee_quat_d(4)*eta_ee_quat(1:3) +...
             cross(eta_ee_quat(1:3),eta_ee_quat_d(1:3));
         