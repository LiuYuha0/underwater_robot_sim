function [w, wdot, vc, a] = InverseDynamics_ric_forw(DH, zita, dzita, PARAM, manipFlag)
%
% Used in InverseDynamics to implement the forward recursion
% 
% [w, wdot, vc, a] = InverseDynamics_ric_forw(DH, zita, dzita, PARAM, manipFlag)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table (include joint pos)
%       zita   dim 6+nx1   system velocities
%       dzita  dim 6+nx1   system accelerations
%       PARAM  struct      parameters for the dynamic simulation
%       manipFlag          
%
% output:
%       [w, wdot, a] see description below
%
% Yuhao Liu   2019/12/30


zita  = CheckVector(zita);
dzita = CheckVector(dzita);

z  = [0 0 1]';
n   = size(DH,1);      % joint number
nu1 = zita(1:3);       % vehicle linear velocity;
nu2 = zita(4:6);       % vehicle angular velocity;
dnu1= dzita(1:3);      % vehicle linear acceleration;
dnu2= dzita(4:6);      % vehicle angular acceleration;
r_  = PARAM.r_;
r_c = PARAM.r_c;

if manipFlag == 1
    dq  = zita(6+1:6+n);       % manip_1 joint velocities
    ddq = dzita(6+1:6+n);      % manip_1 joint accelerations
    R_0_B = PARAM.T_0_B(1:3,1:3);
    r_B0_B = PARAM.T_0_B(1:3,4);
else
    dq  = zita(6+n+1:6+2*n);   % manip_2 joint velocities
    ddq = dzita(6+n+1:6+2*n);  % manip_2 joint accelerations
    R_0_B = PARAM.T_0_B_r(1:3,1:3);
    r_B0_B = PARAM.T_0_B_r(1:3,4);
end

r_B0_0 = R_0_B'*r_B0_B;

w    = zeros(3,n);
wdot = zeros(3,n);
a    = zeros(3,n);
v    = zeros(3,n);
vc   = zeros(3,n);

% variable description:
%   w, wdot, v, vc, a
%   are all 3xn matrices, the i column is the vector of link i
%   all kinematic variables are in their own frame.
%   e.g.: w(:,2) is the angular velocity of the origin of frame 2 expressed in frame 2
%   w    - angular velocity of the frame 
%   wdot - angular acceleration of the frame 
%   a    - linear acceleration of origin of the frame 
%   v    - linear velocity of the frame origin
%   vc   - linear velocity of the center of mass 

w0    = R_0_B'*nu2;
wdot0 = R_0_B'*dnu2;
v0    = R_0_B'*nu1 + cross(w0,r_B0_0);
a0    = R_0_B'*dnu1 + cross(wdot0,r_B0_0) + cross(w0,cross(w0,r_B0_0));


for i=1:n
    R = Rot_dh(DH(i,2),DH(i,4));
    R = R';   
    if i==1
        w(:,i)    = R*( w0 + dq(i)*z );
        wdot(:,i) = R*( wdot0 + cross(w0,dq(i)*z) + ddq(i)*z);
        temp    = R*v0;
        temp2   = cross(w(:,i),r_(:,i));
        v(:,i)  = temp + temp2;
        vc(:,i) = temp + cross(w(:,i),r_c(:,i));
        a(:,i)  = R*a0 + cross(wdot(:,i),r_(:,i)) + cross(w(:,i),temp2);
    else
        w(:,i)    = R*( w(:,i-1) + dq(i)*z );  % 2.64
        wdot(:,i) = R*( wdot(:,i-1) + cross(w(:,i-1),dq(i)*z) + ddq(i)*z); % 2.65
        temp    = R*v(:,i-1);
        temp2   = cross(w(:,i),r_(:,i));
        v(:,i)  = temp + temp2; % 2.66
        vc(:,i) = temp + cross(w(:,i),r_c(:,i)); % 2.67
        a(:,i)  = R*a(:,i-1) + cross(wdot(:,i),r_(:,i)) + cross(w(:,i),temp2); %2.68
   end
end



