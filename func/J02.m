function J = J02(p_obst,eta,DH,T_0_B)
%
% Computes the Jacobian for the task function:
%   end-effector obstacle avoidance
%
% function  J = J02(p_obst,d,eta,DH,T_0_B)
%
% input:
%       p_obst   dim 3x1     obstacle position
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

p_obst = CheckVector(p_obst);
eta    = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);
dist = sqrt((p_obst-eta_ee1)'*(p_obst-eta_ee1));

J = Jacobian(eta,DH,T_0_B);
J = J(1:3,:);
if dist<1e-3
    dist = 1e-3;
end
J = - (p_obst-eta_ee1)'*J/dist;