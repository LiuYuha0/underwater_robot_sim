function dzita = DirectDynamics_d(eta2,DH,DH_r,zita,tau,PARAM)
%
% Computes the direct dynamics
%
% function dzita = DirectDynamics(eta2,DH,zita,tau,PARAM)
%
% input:
%       eta2    dim 3x1     vehicle orientation
%       DH/DH_r dim nx4     Denavit-Hartenberg table (include joint pos)
%       zita    dim 6+nx1   system velocities
%       tau     dim 6+nx1   generalized forces
%       PARAM   struct      parameters for the dynamic simulation
%
% output:
%       dzita   dim 6+nx1   system accelerations
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta2 = CheckVector(eta2);
zita = CheckVector(zita);
tau  = CheckVector(tau);

n = 2*size(DH,1); 

% compute all the dynamic terms but the mass matrix
tau_n = InverseDynamics_d(eta2,DH,DH_r,zita,zeros(6+n,1),PARAM);   

% compute the mass matrix
M = zeros(6+n,6+n);
dummy = PARAM.g0;
PARAM.g0 = [0 0 0]';
for i=1:(6+n)
    e = zeros(6+n,1);
    e(i) = 1;
    % compute inverse dynamic with null velocity, no gravity and "versor"
    % acceleration to obtain the ith column
    M(:,i) = InverseDynamics_d(eta2,DH,DH_r,zeros(6+n,1),e,PARAM);
end
PARAM.g0 = dummy;

l=eig(M);
l(12) = 0;
if l(6+n)<0
    error('DirectDynamics.m : M not positive definite')
end
%fprintf('\n condition number %f',l(1)/l(6+n))
dzita = M\(tau-tau_n); 
%keyboard


