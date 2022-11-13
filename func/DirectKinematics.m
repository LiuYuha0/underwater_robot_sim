function T = DirectKinematics(eta,DH,T_0_B)
%
% Computes the homogeneous transformation matrix 
%	from intertial frame to end-effector
%
% function T = DirectKinematics(eta,DH,T_0_B)
%
% input:
%       eta    dim 6x1     vehicle position/orientation
%       DH     dim nx4     Denavit-Hartenberg table
%       T_0_B  dim 4x4     Homogeneous transformation 
%                          matrix from vehicle to zero frame
%
% output:
%       T      dim 4x4    Homogeneous transformation 
%                         matrix from inertial to end-effector
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta=CheckVector(eta);

n = size(DH,1);     % joint number
eta1 = eta(1:3);
eta2 = eta(4:6);

% from inertial to vehicle-fixed
RBI = Rpy2Rot(eta2);
T = [RBI, eta1; 0 0 0 1];

% from vehicle-fixed to zero
T = T*T_0_B;

% manipulator cycle
for i=1:n
    TT = Homogeneous_dh(DH(i,:));
    T = T*TT;
end
