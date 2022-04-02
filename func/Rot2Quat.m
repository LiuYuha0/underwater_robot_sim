function e = Rot2Quat(R)
%
% From rotation matrix to quaternions
%
% function e = Rot2Quat(R)
%
% input:
%       R		dim 3x3     rotation matrix
%
% output:
%       e       dim 4x1      quaternion
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

e = zeros(4,1);

e(4) = .5*sqrt(R(1,1)+R(2,2)+R(3,3)+1);
%if imag(e(4))>0
%    e(4)=0;
%end

if (R(3,2)-R(2,3))>=0
    e(1) = .5*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
else
    e(1) = -.5*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
end

if (R(1,3)-R(3,1))>=0
    e(2) = .5*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);
else
    e(2) = -.5*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);
end

if (R(2,1)-R(1,2))>=0
    e(3) = .5*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);
else
    e(3) = -.5*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);
end

e=real(e);
e = e/norm(e);

% R4 = R(1,1) + R(2,2) + R(3,3);
% 
% [~,index] = max([R(1,1), R(2,2), R(3,3), R4]);
% 
% if index==1
%    p1 = sqrt(1 + 2*R(1,1) - R4);  
%    p2 = (R(2,1) + R(1,2))/p1;
%    p3 = (R(3,1) + R(1,3))/p1;
%    p4 = (R(3,2) - R(2,3))/p1;
% elseif index==2
%    p2 = sqrt(1 + 2*R(2,2) - R4);  
%    p1 = (R(2,1) + R(1,2))/p2;
%    p3 = (R(3,2) + R(2,3))/p2;
%    p4 = (R(1,3) - R(3,1))/p2;
% elseif index==3
%    p3 = sqrt(1 + 2*R(3,3) - R4);  
%    p1 = (R(3,1) + R(1,3))/p3;
%    p2 = (R(3,2) + R(2,3))/p3;
%    p4 = (R(2,1) - R(1,2))/p3;
% elseif index==4
%    p4 = sqrt(1 + 2*R4 - R4);  
%    p1 = (R(3,2) - R(2,3))/p4;
%    p2 = (R(1,3) - R(3,1))/p4;
%    p3 = (R(2,1) - R(1,2))/p4;
% end
% 
% e(1) = p1/2;
% e(2) = p2/2;
% e(3) = p3/2;
% e(4) = p4/2;
% 
% if e(4)<=0
%     e = -e;
% end
% 
% e = e';
