function DrawCube(p,side_size)
%
% Draw simple cube or parallelepiped
%
% DrawCube(p,side_size)
%
% input:
%   p          dim 3x1    cube center
%   side_size  dim 3x1    side size (dim 1x1 for cube)
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

p         = CheckVector(p);
side_size = CheckVector(side_size);

if size(side_size,1)==1
    side_size(2) = side_size(1);
    side_size(3) = side_size(1);
end


vm = [0            0            0; 
      side_size(1) 0            0; 
      side_size(1) side_size(2) 0; 
      0            side_size(2) 0; 
      0            0            side_size(3); 
      side_size(1) 0            side_size(3); 
      side_size(1) side_size(2) side_size(3); 
      0            side_size(2) side_size(3)];

% the cube is translated so that the variable p is the center 
vm(:,1) = vm(:,1)+p(1)-side_size(1)/2;
vm(:,2) = vm(:,2)+p(2)-side_size(2)/2;
vm(:,3) = vm(:,3)+p(3)-side_size(3)/2;

fm = [1 2 6 5; 
      2 3 7 6; 
      3 4 8 7; 
      4 1 5 8; 
      1 2 3 4; 
      5 6 7 8];

h=patch('Vertices',vm,'Faces',fm);

set(h,'facealpha',.5)
set(h,'edgecolor','white');
colormap(bone)