function R = Quat2Rot(e)
%
% Rotation matrix from body-fixed frame to inertial frame in quaternions
%
% function R = Quat2Rot(e)
%
% input:
%       e		dim 4x1     quaternion
%
% output:
%       R      	dim 3x3      Rotation matrix
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

t1 = e(1)^2;
t2 = e(2)^2;
t3 = e(3)^2;
t4 = e(1)*e(2);
t5 = e(1)*e(3);
t6 = e(2)*e(3);
t7 = e(4)*e(1);
t8 = e(4)*e(2);
t9 = e(4)*e(3);

R = [1-2*(t2+t3)    2*(t4-t9)    2*(t5+t8)
       2*(t4+t9)    1-2*(t1+t3)   2*(t6-t7)
       2*(t5-t8)     2*(t6+t7)   1-2*(t1+t2)];
