function tau = InverseDynamics(eta2,DH,zita,dzita,PARAM)
%
% Computes the inverse dynamics
%
% function tau = InverseDynamics(eta2,DH,zita,dzita,PARAM)
%
% input:
%       eta2   dim 3x1     vehicle orientation
%       DH     dim nx4     Denavit-Hartenberg table (include joint pos)
%       zita   dim 6+nx1   system velocities
%       dzita  dim 6+nx1   system accelerations
%       PARAM  struct      parameters for the dynamic simulation
%
% output:
%       tau    dim 6+nx1   generalized forces
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

% variable description:
%   w, wdot, a
%   are all 3xn matrices, the i column is the vector of link i
%   all kinematic variables are in their own frame.
%   e.g.: w(:,2) is the angular velocity of the origin of frame 2 expressed in frame 2
%   w    - angular velocity of the frame origin
%   wdot - angular acceleration of the frame origin
%   a    - linear acceleration of the frame origin

eta2  = CheckVector(eta2);
zita  = CheckVector(zita);
dzita = CheckVector(dzita);

% forward ricorsion 
[w, wdot, vc, a] = InverseDynamics_ric_forw(DH, zita, dzita, PARAM);
dnu = dzita(1:6);

% backward ricorsion 
tau = InverseDynamics_ric_back(eta2, DH, zita, dnu, w, wdot, vc, a, PARAM);