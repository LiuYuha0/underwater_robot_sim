function J = J07(p_d,eta,DH,T_0_B)
%
% Computes the Jacobian for the task function:
%   end-effector field-of-view "cameraman" error
%
% function  J = J07(a_d,eta,DH,T_0_B)
%
% input:
%       p_d      dim 3x1     point to be framed
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       J        dim 1x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

p_d = CheckVector(p_d);
eta = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);
a       = TT(1:3,3);
r       = p_d - eta_ee1;
a_d     = r/norm(r);

sigma = sqrt((a_d-a)'*(a_d-a));
J = Jacobian(eta,DH,T_0_B);
if sigma<1e-3
    sigma = 1e-3;
end
J = (a_d-a)'*(-S(a_d)*pinv(S(r))*J(1:3,:) + S(a)*J(4:6,:))/(sigma);
