function J = J04(eta,DH,T_0_B)
%
% Computes the Jacobian for the task function:
%   end-effector orientation error
%
% function  J = J04(eta,DH,T_0_B)
%
% input:
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       J        dim 3x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta      = CheckVector(eta);

J = Jacobian(eta,DH,T_0_B);
J = J(4:6,:);
