function DrawUVDMS_ori(eta,DH,DH_r,PARAM)
%
% 3D rendering of the UVDMS in a given configuration
% only the pose displayed
%
% DrawUVDMS_eta(eta,DH,DH_r,PARAM)
%
% input:
%   eta   dim 6x1    vehicle position/orientation
%   DH    dim nx4    DH table left
%   DH_r  dim nx4    DH table right
%   PARAM struct     system parameters
%
% output:
%
%   generate figure
%
% Yuhao Liu   2019/12/30

eta=CheckVector(eta);

a1 = PARAM.a1;
a2 = PARAM.a2;
a3 = PARAM.a3;

% homogeneous matrix for the inertial frame
T_i = [eye(3,3), [0 0 0]'
        0 0 0 1];

hold on
grid on

% DrawFrame(T_i,1,.35);
DrawVehicle_ori(eta, [a1;a2;a3],PARAM.type);
DrawRobot(eta,DH,PARAM.T_0_B);

% T_i_n = DirectKinematics(eta,DH,PARAM.T_0_B);
% DrawFrame(T_i_n,0,.35);

DrawRobot(eta,DH_r,PARAM.T_0_B_r);

% T_i_n_r = DirectKinematics(eta,DH_r,PARAM.T_0_B_r);
% DrawFrame(T_i_n_r,0,.35);


view([1 3 .5])
axis('equal')
xlabel('x'),ylabel('y'),zlabel('z')
set(gca,'XDir','reverse','ZDir','reverse');





