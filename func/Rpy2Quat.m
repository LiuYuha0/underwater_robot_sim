function e = Rpy2Quat(rpy)
%
% From Euler angles (roll-pitch-yaw) to quaternions
%
% function e = Rpy2Quat(rpy)
%
% input:
%       rpy     dim 3x1     roll-pitch-yaw
%
% output:
%       e       dim 4x1      quaternion
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

R = Rpy2Rot(rpy);
e = Rot2Quat(R);

