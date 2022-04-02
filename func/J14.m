function J = J14(p_obst,eta,DH)
%
% Computes the Jacobian for the task function:
%   vehicle obstacle avoidance
%
% function  J = J14(p_obst,eta,DH)
%
% input:
%       p_obst   dim 3x1     obstacle position
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%
% output:
%       J        dim 1x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

p_obst = CheckVector(p_obst);
eta    = CheckVector(eta);

n = size(DH,1);
eta_1 = eta(1:3);
R = Rpy2Rot(eta(4:6));
sigma = sqrt((p_obst-eta_1)'*(p_obst-eta_1));

if sigma<1e-3
    sigma = 1e-3;
end
J = [- (p_obst-eta_1)'*R/sigma zeros(1,3+n)];