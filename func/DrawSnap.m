function DrawSnap(eta,DH,DH_r,PARAM,depth,oriOnly)
%
% 3D rendering of the UVMS in a given scenario
%
% DrawSnap(eta,DH,PARAM,depth)
%
% input:
%   eta     dim 6x1    vehicle position/orientation
%   DH/DH_r dim nx4    DH table (contain joint positions)
%   PARAM   struct     system parameters
%   depth   dim 1x1    sea bottom depth
%   oriOnly dim 1    draw orientation only 
%
% output:
%
%   generate figure
%
% Yuhao Liu   2019/12/30


% draw UVMS
if(oriOnly == 1)
    DrawUVDMS_ori(eta,DH,DH_r,PARAM);
else
    DrawUVDMS(eta,DH,DH_r,PARAM);
end
axis tight
v = axis;
dimx = (v(2)-v(1));
dimy = 2*(v(3)-v(4));

% generate sea bottom
DrawTexture([dimx dimy],[v(1)-0*dimx/4 v(4)-dimy/4 depth],0.5,'texture3.jpg')

% generate sea surface (always at -1 m)
DrawTexture([dimx dimy],[v(1)-0*dimx/4 v(4)-dimy/4 -1],0.5,'texture4.jpg')

axis tight


