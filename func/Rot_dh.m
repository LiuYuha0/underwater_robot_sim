function R = Rot_dh(alpha,theta)
%
% Rotation matrix between consecutive frames according to DH convention
%
% function R = Rot_dh(alpha,theta)
%
% input:
%       alpha     dim 1x1     Denavit-Hartenberg parameter
%       theta     dim 1x1     Denavit-Hartenberg parameter
%
% output:
%       R      dim 3x3      Rotation matrix
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

R = [ct -st*ca st*sa;
    st ct*ca -ct*sa;
    0 sa ca];