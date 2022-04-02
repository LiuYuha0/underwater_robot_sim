function J = Jacobian(eta,DH,T_0_B)
%
% Computes the Jacobian from zita to
% end-effector linear and angular velocities
% expressed in the inertial frame
% 计算从zita到末端执行器在惯性框架中表示的线速度和角速度的雅可比行列式
% function J = Jacobian(eta,DH,T_0_B)
%
% input:
%       eta    dim 6x1     vehicle position/orientation
%       DH     dim nx4     Denavit-Hartenberg table
%       T_0_B  dim 4x4     Homogeneous transformation 
%						   matrix from vehicle to zero frame
%
% output:
%       J      dim 6x(6+n) Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta1 = CheckVector(eta(1:3));
eta2 = CheckVector(eta(4:6));
n = size(DH,1);

r_B0_B = T_0_B(1:3,4);
R_0_B  = T_0_B(1:3,1:3);
J = zeros(6,6+n);

Jman = J_man(DH);
R_B_I = Rpy2Rot(eta2);
R_0_I = R_B_I*R_0_B;

T = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = T(1:3,4);
r_B0_I = R_B_I*r_B0_B;
eta_0ee_I = eta_ee1 - eta1 - r_B0_I;

% Position Jacobian
J(1:3,1:3)   = R_B_I;
J(1:3,4:6)   = -(S(r_B0_I)+S(eta_0ee_I))*R_B_I;
J(1:3,7:6+n) = R_0_I*Jman(1:3,:);

% Orientation Jacobian
J(4:6,1:3)   = zeros(3,3);
J(4:6,4:6)   = R_B_I;
J(4:6,7:6+n) = R_0_I*Jman(4:6,:);
