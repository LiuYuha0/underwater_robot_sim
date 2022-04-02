function T = Homogeneous_dh(DH_i)
%
% Homogeneous transformation matrix between consecutive frames according to DH convention
%
% T = Homogeneous_dh(DH_i)
%
% input:
%       DH_i     dim 1x4     row i of the Denavit-Hartenberg table
%
% output:
%       T      dim 4x4      Homogeneous transformation matrix
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

a     = DH_i(1);
alpha = DH_i(2);
d     = DH_i(3);
theta = DH_i(4);

R = Rot_dh(alpha,theta);
p = [a*cos(theta);
    a*sin(theta);
    d];

T = [R p;
    0 0 0 1];