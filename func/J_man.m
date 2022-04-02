function J = J_man(DH)
%
% Computes the geometric Jacobian for a manipulator
% with rotational-only joints with respect to 
% the zero(base) frame
%
% function J = J_man(DH)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table
%
% output:
%       J      dim 6xn     Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

n = size(DH,1);
J = zeros(6,n);

% compute homog. transf. from base frame
T0 = zeros(4,4,n);
T0(:,:,1) = Homogeneous_dh(DH(1,:));
% p: 3xn matrix, the generic column is the position of frame i expressed in inertial frame
p(:,1) = T0(1:3,4,1);
% z: 3xn matrix, the generic column is the z-versor of frame i expressed in inertial frame
z(:,1) = T0(1:3,3,1);
for i=2:n
    T_i = Homogeneous_dh(DH(i,:));
    T0(:,:,i) = T0(:,:,i-1)*T_i;
    p(:,i) = T0(1:3,4,i);
    z(:,i) = T0(1:3,3,i);
end

z0 = [0 0 1]';
p0 = [0 0 0]';
J(:,1) = [cross(z0,p(:,n)-p0);
            z0];
for i=2:n
    J(:,i) = [cross(z(:,i-1),p(:,n)-p(:,i-1));
            z(:,i-1)];
end





