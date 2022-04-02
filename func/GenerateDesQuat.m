function quat_d = GenerateDesQuat(quat_i,R_f,dq_c,tf,t)
%
% Generate current desired quaternion by assigning desired final rotation 
% 通过分配所需的最终旋转来生成当前四元数
%
% quat_d = GenerateDesQuat(quat_i,R_f,qd_c,tf,t)
%
% input:
%       quat_i  dim 4x1     initial quaternion
%       R_f     dim 3x3     final roation matrix
%       dq_c    dim 1       cruise velocity
%       tf      dim 1       final time
%       t       dim 1       output time
%
% output:
%       quat_d  dim 4x1     desired quaternion
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

quat_i = CheckVector(quat_i);

% generate initial rotation
R_i = Quat2Rot(quat_i);

% generate rotation required to align R_i to R_f
R = R_i'*R_f;

% extract quaternion
quat = Rot2Quat(R);

if (quat(4)==1)
    quat_d = quat_i;    % no rotation required
else
    % extract axis-angle
    r = 2*asin(quat(1:3));
    r = r/norm(r);
    theta = 2*acos(quat(4));
    % generate time law for theta from zero
    theta = trapezoidal2(0,theta,dq_c,tf,t);
    % quaternion at time t
    quat_t = [r*sin(theta/2);
                cos(theta/2)];
    % this needs to be composed with quat_i
    quat_d = [quat_i(4)*quat_t(1:3)+quat_t(4)*quat_i(1:3)+cross(quat_i(1:3),quat_t(1:3));
              quat_i(4)*quat_t(4)-quat_i(1:3)'*quat_t(1:3)];
    if (quat_d(4)<0)
        quat_d =  - quat_d;
    end
end        
