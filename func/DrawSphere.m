function DrawSphere(p,size)
%
% DrawSphere(p,size)
%
% input:
%   p     dim 3x1    sphere center
%   size  dim 1x1    radium
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

p=CheckVector(p);

% generate points on the surface of a unitary sphere centered in the origin
[x,y,z] = sphere(40);

x = p(1) + size*x;
y = p(2) + size*y;
z = p(3) + size*z;

h = surfl(x,y,z,[0 0 -5]);
set(h,'facealpha',.5)
set(h,'facecolor','interp');
set(h,'edgecolor','none');
colormap(bone)
