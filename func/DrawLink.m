function DrawLink(pa,pb,d)
%
% DrawLink(pa,pb)
%
% input:
%   pa   dim 3x1    "starting" point
%   pb   dim 3x1    "ending" point
%   d    dim 1x1    link diameter
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

pa=CheckVector(pa);
pb=CheckVector(pb);

% generate points on the cylinder
% aligned with x
[z,y,x] = cylinder(d*ones(41,1),40);
x = norm(pb-pa)*x;
x2 = (pb-pa)/norm(pb-pa);
if ((x2(1)==-1)||(x2(1)==1))
    z2 = [0 0 1]';
else
    z2 = cross([1;0;0],x2); z2 = z2/norm(z2);
end
y2 = cross(x2,z2);

% rotate and translate points
R = [x2 y2 z2];
for i=1:length(x)
   for j=1:length(x)
      % rotation
      rr=R*[x(i,j) y(i,j) z(i,j)]';
      x(i,j) = rr(1);
      y(i,j) = rr(2); 
      z(i,j) = rr(3);
      % translation
      x(i,j) = x(i,j) + pa(1);
      y(i,j) = y(i,j) + pa(2); 
      z(i,j) = z(i,j) + pa(3);
   end
end


h = surfl(x,y,z,[0 0 -5]);
set(h,'facealpha',.5)
set(h,'facecolor','interp');
set(h,'edgecolor','none');
colormap(bone)


