function DrawVehicle(eta,a,type)
%
% Draw a "minimalistic" shape for the vehicle
%
% DrawVehicle(eta,a,type)
%
% input:
%   eta   dim 6x1    vehicle position/orientation
%   a     dim 3x1    semi-axes of the vehicle/ellipsoid
%   type  string     shape: 'ellipse' or 'parallelepiped'
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta1 = CheckVector(eta(1:3));
eta2 = CheckVector(eta(4:6));
a    = CheckVector(a);

if nargin==2
    type = 'ellipse';
end

% generate points on the surface of a unitary sphere centered in the origin
if strcmp(type,'ellipse')
    [x,y,z] = sphere(40);
elseif strcmp(type,'Romeo')
    [x,y,z] = superquad(.2,.2,40);
else
    fprintf('\n WARNING in DrawVehicle.m: vehicle type unrecognized, ellipse used'); 
    [x,y,z] = sphere(40);
end

% ellipsoid with given principal axes
x = a(1)*x;
y = a(2)*y;
z = a(3)*z;

% rotate and translate points
RBI = Rpy2Rot(eta2);
% T = [RBI, eta1; 0 0 0 1];
% DrawFrame(T,0,.35);
for i=1:length(x)
   for j=1:length(x)
      % rotation
      rr=RBI*[x(i,j) y(i,j) z(i,j)]';
      x(i,j) = rr(1);
      y(i,j) = rr(2); 
      z(i,j) = rr(3);
      % translation
      x(i,j) = x(i,j) + eta1(1);
      y(i,j) = y(i,j) + eta1(2); 
      z(i,j) = z(i,j) + eta1(3);    
   end
end


h = surfl(x,y,z,[0 0 -5]);
set(h,'facealpha',.5)
set(h,'facecolor','interp');
set(h,'edgecolor','none');
colormap(bone)


