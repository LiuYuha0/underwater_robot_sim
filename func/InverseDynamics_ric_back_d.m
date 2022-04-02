function tau = InverseDynamics_ric_back_d(eta2, DH, DH_r, zita, dnu, w, wdot, vc, a, w_r, wdot_r, vc_r, a_r, PARAM)
%
% Used in InverseDynamics to implement the backward recursion
%
% function tau = InverseDynamics_ric_back(eta2, DH, zita, dnu, w, wdot, vc, a, PARAM)
%
% input:
%       eta2    dim 3x1     vehicle orientation
%       DH/DH_r dim nx4     Denavit-Hartenberg table
%       zita    dim 6+nx1   system velocities
%       dnu     dim 6x1     vehicle linear and angular accelerations
%       w, wdot, vc, a      see below
%       PARAM   struct      parameters for the dynamic simulation
%
% output:
%       tau    dim 6+nx1   generalized forces
%
% Yuhao Liu   2019/12/30

n   = size(DH,1);     % joint number
nu  = zita(1:6);
dq  = zita(6+1:6+6);
tau = zeros(6+6,1);

% variable description:
%   w, wdot, v, vc, a
%   are all 3xn matrices, the i column is the vector of link i
%   all kinematic variables are in their own frame.
%   e.g.: w(:,2) is the angular velocity of the origin of frame 2 expressed in frame 2
%   w    - angular velocity of the frame
%   wdot - angular acceleration of the frame
%   a    - linear acceleration of origin of the frame

% remember (see file InverseDynamics_forces.png):
%   link i connects joint i with joint i+1
%   frame i-1 is positioned along joint i
%   frame i   is positioned along joint i+1
%   force/moment/torque i   are acting on joint i   -> origin of frame i-1
%   force/moment/torque i+1 are acting on joint i+1 -> origin of frame i

T_0_B  = PARAM.T_0_B;
r_B0_B = PARAM.T_0_B(1:3,4);
T_0_B_r  = PARAM.T_0_B_r;
r_B0_B_r = PARAM.T_0_B_r(1:3,4);
r_  = PARAM.r_;
r_c = PARAM.r_c;
r_b = PARAM.r_b;
r_k = PARAM.r_k;
g0  = PARAM.g0;
delta = PARAM.delta;
m   = PARAM.m;
M   = PARAM.M;
Ma  = PARAM.Ma;
I   = PARAM.I;
Ia  = PARAM.Ia;
fric_dry = PARAM.fric_dry;
fric_vis = PARAM.fric_vis;
Ds   = PARAM.Ds;
CD   = PARAM.CD;
rho  = PARAM.rho;

% Compute all the homogeneous with respect to the inertial frame
% for the gravity-related forces
RBI = Rpy2Rot(eta2);        % from vehicle-fixed to inertial
R = RBI*T_0_B(1:3,1:3);     % from zero to intertial
RiI = zeros(3,3,n);
RiI(:,:,1) = R*Rot_dh(DH(1,2),DH(1,4));
% manipulator cycle
for i=2:n
    RR = Rot_dh(DH(i,2),DH(i,4));
    RiI(:,:,i) = RiI(:,:,i-1)*RR;
end
clear T % same variable also for torques...

f = zeros(3,n);
t = zeros(3,n);

% start of backward recursion
for i=3:-1:1
    % inertial forces "alone"
    ac = a(:,i) + cross(wdot(:,i),r_k(:,i)) + cross(w(:,i),cross(w(:,i),r_k(:,i)));
    F = (M(:,:,i)+Ma(:,:,i))*ac;
    T = (I(:,:,i)+Ia(:,:,i))*wdot(:,i) + cross(w(:,i),(I(:,:,i)+Ia(:,:,i))*w(:,i));
    
    % gravity and buoyancy
    F_grav = m(i)*RiI(:,:,i)'*g0;
    F_buoy = rho*delta(i)*RiI(:,:,i)'*g0;
    
    % hydrodynamic forces and moments 
    [F_hyd, T_hyd] = Hydro(vc(:,i), Ma(:,:,i), Ds, CD, rho);

    % force/moment transformation from frame i+1
    if i~=n
        % calculate rotation matrix from frame i+1 to frame i
        R_ip1_i = Rot_dh(DH(i+1,2),DH(i+1,4));
        [f(:,i), t(:,i)] = FrameTransfForce(R_ip1_i, f(:,i+1), t(:,i+1), r_(:,i));
    end
    
	% putting all together: forces and moments on link i
    f(:,i) = f(:,i) + F + F_hyd - F_grav + F_buoy;
    t(:,i) = t(:,i) + cross(r_c(:,i), F + F_hyd - F_grav) + cross(r_b(:,i), F_buoy) + T + T_hyd;
   
    % calculate rotation matrix from frame i to frame i-1
    R_i_im1 = Rot_dh(DH(i,2),DH(i,4));
   
    % compute the manipulator torques
    temp2 = R_i_im1*t(:,i);
    tau(6+i) = temp2(3);	
   
    % add viscous and dry friction
   	tau(6+i) = tau(6+i) + fric_dry(i)*sign(dq(i)) + fric_vis(i)*dq(i);
end


RBI_r = Rpy2Rot(eta2);        
R_r = RBI_r*T_0_B_r(1:3,1:3);    
RiI_r = zeros(3,3,n);
RiI_r(:,:,1) = R_r*Rot_dh(DH_r(1,2),DH_r(1,4));

for i=2:n
    RR_r = Rot_dh(DH_r(i,2),DH_r(i,4));
    RiI_r(:,:,i) = RiI_r(:,:,i-1)*RR_r;
end
clear T 

f_r = zeros(3,n);
t_r = zeros(3,n);
for i=n:-1:1
    ac = a_r(:,i) + cross(wdot_r(:,i),r_k(:,i)) + cross(w_r(:,i),cross(w_r(:,i),r_k(:,i)));
    F = (M(:,:,i)+Ma(:,:,i))*ac;
    T = (I(:,:,i)+Ia(:,:,i))*wdot_r(:,i) + cross(w_r(:,i),(I(:,:,i)+Ia(:,:,i))*w_r(:,i));
    
    F_grav_r = m(i)*RiI_r(:,:,i)'*g0;
    F_buoy_r = rho*delta(i)*RiI_r(:,:,i)'*g0;
    
    [F_hyd_r, T_hyd_r] = Hydro(vc_r(:,i), Ma(:,:,i), Ds, CD, rho);

    if i~=n
        R_ip1_i_r = Rot_dh(DH_r(i+1,2),DH_r(i+1,4));
        [f_r(:,i), t_r(:,i)] = FrameTransfForce(R_ip1_i_r, f_r(:,i+1), t_r(:,i+1), r_(:,i));
    end
    
    f_r(:,i) = f_r(:,i) + F + F_hyd_r - F_grav_r + F_buoy_r;
    t_r(:,i) = t_r(:,i) + cross(r_c(:,i), F + F_hyd_r - F_grav_r) + cross(r_b(:,i), F_buoy_r) + T + T_hyd_r;
   
    R_i_im1_r = Rot_dh(DH_r(i,2),DH_r(i,4));
   
    temp2_r = R_i_im1_r*t_r(:,i);
    tau(9+i) = temp2_r(3);	
   
   	tau(9+i) = tau(9+i) + fric_dry(i)*sign(dq(3+i)) + fric_vis(i)*dq(3+i);
end


% calculate force and moments of link 1 in frame O
R_1_0 = R_i_im1;
[temp_sigma_f, temp_sigma_m] = FrameTransfForce(R_1_0, f(:,1), t(:,1), [0 0 0]);

% calculate force and moments of link 0 in frame B
R_0_B = T_0_B(1:3,1:3);
[sigma_f_l, sigma_m_l] = FrameTransfForce(R_0_B, temp_sigma_f, temp_sigma_m, r_B0_B);


% calculate force and moments of link 1 in frame O
R_1_0_r = R_i_im1_r;
[temp_sigma_f_r, temp_sigma_m_r] = FrameTransfForce(R_1_0_r, f_r(:,1), t_r(:,1), [0 0 0]);

% calculate force and moments of link 0 in frame B
R_0_B_r = T_0_B_r(1:3,1:3);
[sigma_f_r, sigma_m_r] = FrameTransfForce(R_0_B_r, temp_sigma_f_r, temp_sigma_m_r, r_B0_B_r);

sigma_f = sigma_f_l + sigma_f_r;
sigma_m = sigma_m_l + sigma_m_r;

% force/moment acting on the vehicle
tau(1:6) = InverseDynamics_vehicle_d(Rpy2Quat(eta2), nu, dnu, rho, g0) + [sigma_f; sigma_m];
