function J = J13(eta_1d,eta,DH)
%
% Computes the Jacobian for the task function:
%   vehicle error position norm
%
% function  J = J13(eta_1d,eta,DH)
%
% input:
%       eta_1d   dim 3x1     vehicle desired position
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%
% output:
%       J        dim 1x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta_1d = CheckVector(eta_1d);
eta    = CheckVector(eta);

n = size(DH,1);
eta_1 = eta(1:3);
R = Rpy2Rot(eta(4:6));
sigma = sqrt((eta_1d-eta_1)'*(eta_1d-eta_1));

if sigma<1e-3
    sigma = 1e-3;
end
J = [- (eta_1d-eta_1)'*R/sigma zeros(1,3+n)];