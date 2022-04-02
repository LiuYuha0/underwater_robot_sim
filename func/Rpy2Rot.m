function R = Rpy2Rot(rpy)
%
% Rotation matrix from body-fixed frame to inertial frame in roll-pitch-yaw
% angles
%
% function R = Rpy2Rot(rpy)
%
% input:
%       rpy     dim 3x1     roll-pitch-yaw angles
%
% output:
%       R      dim 3x3      Rotation matrix
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

phi   = rpy(1);
theta = rpy(2);
psi   = rpy(3);

cp = cos(psi);		sp = sin(psi);
ct = cos(theta);	st = sin(theta);
cf = cos(phi);		sf = sin(phi);

R = [ cp*ct -sp*cf+cp*st*sf  sp*sf+cp*cf*st
       sp*ct  cp*cf+sf*st*sp -cp*sf+st*sp*cf
        -st     ct*sf              ct*cf     ];
