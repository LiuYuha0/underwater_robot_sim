function J = J06(a_d,eta,DH,T_0_B)
%
% Computes the Jacobian for the task function:
%   end-effector field-of-view error
%
% function  J = J06(a_d,eta,DH,T_0_B)
%
% input:
%       a_d      dim 3x1     end-effector desired "a" versor
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

a_d = CheckVector(a_d);
eta = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
a  = TT(1:3,3);
sigma = sqrt((a_d-a)'*(a_d-a));
J = Jacobian(eta,DH,T_0_B);
if sigma<1e-3
    sigma = 1e-3;
end
J = (a_d-a)'*(S(a)*J(4:6,:))/(sigma);
