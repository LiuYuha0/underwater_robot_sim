function DrawFrame(T,mytext,scale)
%
% DrawFrame(T,mytext,scale)
%
% input:
%   T   dim 4x4    homogeneous transf. matrix
%   mytext         ==1 writes "x", "y", "z" close to the versor
%   scale          scaling factor with respect to the versor
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

d_ax = .05*scale; % axis diameter
d_ar = .1*scale;  % axis diameter of the arrow head
d_txt = 1.1;      % text scaling factor

T(1:3,1:3) = scale*T(1:3,1:3);

% axis x - red
DrawArrow(T(1:3,4),T(1:3,4)+T(1:3,1),d_ax,d_ar,[1 0 0]);
if mytext==1
    text(d_txt*(T(1,4)+T(1,1)),d_txt*T(2,4)+T(2,1),d_txt*T(3,4)+T(3,1),'x');
end
% axis y - green
% size is multiplied by .98 so that, when y and z are superimposed, the
% versor z is shown instead of y
DrawArrow(T(1:3,4),T(1:3,4)+T(1:3,2),.98*d_ax,.98*d_ar,[0 1 0]);
if mytext==1
    text(d_txt*(T(1,4)+T(1,2)),d_txt*T(2,4)+T(2,2),d_txt*T(3,4)+T(3,2),'y');
end
% axis z - blue
DrawArrow(T(1:3,4),T(1:3,4)+T(1:3,3),d_ax,d_ar,[0 0 1]);
if mytext==1
    text(d_txt*(T(1,4)+T(1,3)),d_txt*T(2,4)+T(2,3),d_txt*T(3,4)+T(3,3),'z');
end
