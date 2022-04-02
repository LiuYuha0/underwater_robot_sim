%
% HUST-UVDMS
%
% Yuhao Liu   2019/12/30
% Huazhong University of Science and Technology
% 
% Model parameters for HUST-UVDMS (underwater vehicle dual-manipulator system)

fprintf('\n hust UVDMS \n')
fprintf('\n Description of the model')

%------------------------
% vehicle parameters 
% here only the geometric data, the remaining in function
% that computes that vehicle dyanmics
%------------------------
PARAM.type = 'Romeo';

%  rotation matrix from the left manipulator-base frame (zero) to the vehicle-fixed
R_0_B = [1  0  0
         0  0  1
         0  1  0];

r_B0_B = [-0.3 0.36 0.1]'; % vector position from origin of frame B to the
             		 % origin of frame O, expressed in frame B [m]
T_0_B = [R_0_B, r_B0_B; 0 0 0 1];

% rotation matrix from the right manipulator-base frame (zero) to the vehicle-fixed
R_0_B_r = [1  0  0
           0  0  1
           0  1  0];
r_B0_B_r = [-0.3 -0.36 0.1]';
T_0_B_r = [R_0_B_r, r_B0_B_r; 0 0 0 1]; 

% semi-axis for the ellipsoid representing the vehicle
L  = 2.7;   % [m]
a1 = L/2;		% vehicle-fixed x
a2 = .36;		% vehicle-fixed y
a3 = .17;		% vehicle-fixed z

PARAM.T_0_B = T_0_B;
PARAM.T_0_B_r = T_0_B_r;
PARAM.a1 = a1;
PARAM.a2 = a2;
PARAM.a3 = a3;

%------------------------
% manipulator parameters 
%------------------------

% D-H table 
DH_a     = [ 0.15    0.27    0.672  ]';   % [m]
DH_alpha = [pi/2       0     -pi/2  ]';   % [rad]
DH_d     = [ 0         0        0   ]'; % [m]
DH_theta = [0 0 0]'/180*pi; % [rad]
DH = [DH_a DH_alpha  DH_d  DH_theta];

DH_a_r     = [ 0.15   0.27    0.672  ]';  
DH_alpha_r = [pi/2     0      -pi/2  ]';   
DH_d_r     = [ 0       0        0   ]'; 
DH_theta_r = [0 0 0]'/180*pi; 
DH_r = [DH_a_r DH_alpha_r  DH_d_r  DH_theta_r];

% r_ matrix 3xn with in column the vector position
%   from origin of frame i-1 to origin of frame i expressed in frame i.
r_  = [0.15  .27  .6  
       0      0    0  
       0      0    0  ];      % m

% r_c matrix 3xn with in column the vector position
%   from origin of frame i-1 to the center of mass of link i expressed in frame i.
r_c  = [0.075    0.14   0.335 
          0       0      0     
          0      0.05      0    ];      % m

% r_b matrix 3xn with in column the vector position
%   from origin of frame i-1 to the center of buoyancy of link i expressed in frame i.
r_b  = [0.075    0.14   0.355 
        0         0      0    
        0         0      0     ];      % m
      
% r_k matrix 3xn with in column the vector position
%   from origin of frame i to the center of mass of link i expressed in frame i.
r_k = r_c - r_;

g0 = [0 0 9.81]';   % gravity acceleration in inertial frame
rho = 1000;		    % water density [kg/m^3]

r_l = [.07 .07 .07 ];  % link radius [m]
L_l = [.15 .27 .672];  % link length [m]
delta = pi*(r_l.^2).*(L_l);   % link volumes [m^3]
m = [9.408 8.354 12.362];      % link masses [kg]

% each link is modeled as a cylinder so the 6x6 added matrix is diagonal
% the asimmetric term of the added masses and inertias is the direction
% of the cylinder length

% M is a 3x3xn matrix where M(:,:,i) is the mass of link i
M = zeros(3,3,3);
for i=1:3
    M(:,:,i) = m(i)*eye(3,3);
end
% Ma is a 3x3xn matrix where Ma(:,:,i) is the added mass of link i
Ma = zeros(3,3,3);
Ma(:,:,1) = diag([ rho*delta(1), 0.1*m(1), rho*delta(1)]);
Ma(:,:,2) = diag([ 0.1*m(2), rho*delta(2), rho*delta(2)]);
Ma(:,:,3) = diag([ rho*delta(3), 0.1*m(3), rho*delta(3)]);

% I is a 3x3xn matrix where I(:,:,i) is the inertia of link i
I = zeros(3,3,3);
I(:,:,1) = 20*diag([5 1.5 5]);
I(:,:,2) = 20*diag([1 4 4]);
I(:,:,3) = 20*diag([.1 .025 .1]);
% Ia is a 3x3xn matrix where Ia(:,:,i) is the added inertia of link i
Ia = zeros(3,3,3);
Ia(:,:,1) = diag([(pi*rho*r_l(1)^2*L_l(1)^2)/12, 0, (pi*rho*r_l(1)^2*L_l(1)^2)/12]);
Ia(:,:,2) = diag([0, (pi*rho*r_l(2)^2*L_l(2)^2)/12, (pi*rho*r_l(2)^2*L_l(2)^2)/12]);
Ia(:,:,3) = diag([(pi*rho*r_l(3)^2*L_l(3)^2)/12, 0, (pi*rho*r_l(3)^2*L_l(3)^2)/12]);


% dry friction of the links
fric_dry = 0*[20 10 5 ];
% viscous friction of the links
fric_vis = [30 20 5 ];

Ds = 0.4;   % linear skin coefficient
CD = 0.6;   % quadratic drag coefficient
CL = 0;		% lift coefficient

zita_limit = [.05 .05 .05 .05 .5 .05 [100 10 10]/180*pi];
q_nominal = [90 0 0]/180*pi;


PARAM.r_  = r_;
PARAM.r_c = r_c;
PARAM.r_b = r_b;
PARAM.r_k = r_k;
PARAM.rho = rho;
PARAM.g0  = g0;
PARAM.delta  = delta;
PARAM.m   = m;
PARAM.M   = M;
PARAM.Ma  = Ma;
PARAM.I   = I;
PARAM.Ia  = Ia;
PARAM.fric_dry = fric_dry;
PARAM.fric_vis = fric_vis;
PARAM.Ds = Ds;
PARAM.CD = CD;
PARAM.CL = CL;
PARAM.zita_limit = zita_limit;
PARAM.q_nominal = q_nominal;

clear i L rv_g rv_b a1 a2 a3 R_0_B R_0_B_r r_B0_B r_B0_B_r T_B_0 T_B_0_r rho g0
clear DH_a DH_alpha DH_d DH_theta r_k r_c r_b r_ m 
clear r_l L_l delta
clear M Ma I Ia 
clear fric_dry fric_vis Ds CD CL zita_limit q_nominal

PrintData(DH,PARAM);
PrintData(DH_r,PARAM);