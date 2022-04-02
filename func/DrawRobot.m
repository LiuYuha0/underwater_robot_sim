function DrawRobot(eta,DH,T_0_B)
%
% Draw a robot in 3D with intermediate frames
%
% function DrawRobot(eta,DH,T_0_B)
%
% input:
%   eta   dim 6x1    vehicle position/orientation
%   DH    dim nx4    DH table (contain joint positions)
%   T_0_B dim 4x4    hom. transf. from vehicle-fixed to zero frame
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta=CheckVector(eta);

n = size(DH,1);
eta1 = eta(1:3);
% eta1 = zeros(3,1);
eta2 = eta(4:6);
TI = zeros(4,4,n);  % contain the n homogeneous to the inertial frame

hold on

% from inertial to vehicle-fixed
RBI = Rpy2Rot(eta2);
% T_I_B = [RBI, eta1; 0 0 0 1];
T_I_B = [RBI, eta1; 0 0 0 1];

% from inertial to vehicle-fixed to zero
T_I_0 = T_I_B*T_0_B;
% DrawFrame(T_I_0,0,.2);

% manipulator cycle
TI(:,:,1) = T_I_0*Homogeneous_dh(DH(1,:));
for i=2:n
    TI(:,:,i) = TI(:,:,i-1)*Homogeneous_dh(DH(i,:));
end

% draw frame for each link
for i=1:n
%     DrawFrame(TI(:,:,i),0,.2);
end

% link diameter decresing
d = .05;
DrawLink(T_I_0(1:3,4),T_I_B(1:3,4),d);
DrawLink(T_I_0(1:3,4),TI(1:3,4,1),d)
for i=2:n
    d=.8*d;
    DrawLink(TI(1:3,4,i-1),TI(1:3,4,i),d)
end
