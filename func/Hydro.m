function [f,m] = Hydro(vc, Ma, Ds, CD, rho)
%
% Compute hydrodinamics forces and moments for the generic link i
%  linear skin, quadratic drag force and munk moment
%  no lift forces and vortex shedding!!!!
%  projected frontal area = 1
%
% function [f,m] = Hydro(vc, Ma, Ds, CD, rho)
%
% input:
%       vc      dim 3x1     linear velocity of the center of mass 
%			                of the generic link in its own frame
%       Ma      dim 3x3     added mass
%       Ds      dim 1x1     
%       CD      dim 1x1     
%       rho     dim 1x1     
%
% output:
%       f		dim 3x1 - hydrodinamics forces
%       m		dim 3x1 - hydrodinamics moments
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv


U = norm(vc,2);
if U==0
    f = zeros(3,1);
    m = zeros(3,1);
else
    vc_versor = (1/U)*vc;
    f_s = Ds*vc;                    % linear skin
    f_d = 0.5*rho*CD*U^2*vc_versor;	% quadratic drag
    f = (f_s+f_d);			
    m = cross(vc,Ma*vc);
end
