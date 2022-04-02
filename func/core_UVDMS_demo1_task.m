% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

%------------------------
% simulation parameters 
%------------------------
t_f = 1;       % final simulation time [s]
Ts  = 0.01;	% sampling time [s]
t   = 0:Ts:t_f;	% time vector [s]
npti = length(t);

KinOnly   = 1;  % kinematic simulation only, controller needs to provide reference velocities
                % if null the simulation is dynamic and the controller
                % outputs the forces/moments/torques
Graphics  = 1;	% provide or not graphics in the end of the simulation

% Definition of the variables
n   = size(DH,1);
eta           = zeros(6,npti);
q             = zeros(n,npti);
q_r           = zeros(n,npti);
zita          = zeros(6+n,npti);
eta_ee1       = zeros(3,npti);
eta_ee_quat   = zeros(4,npti);
zita_d        = zeros(6+n,npti);
eta_ee1_d     = zeros(3,npti);
eta_ee_quat_d = zeros(4,npti);
dzita = zeros(6+n,npti);
tau   = zeros(6+n,npti);
X     = zeros(1,npti);
Y     = zeros(1,npti);
Z     = zeros(1,npti);

plate1 = [0.7  1 -0.5];   
plate2 = [0.7 -1 -0.5];  %圆柱体两个端面中心点坐标    
radius = 0.1;  %端面半径
mesh_n = 20; %精度
cylinder_color = [0.6 0.6 0.6];  % RGB三通道
cylinder_closed = 1; %1：封闭圆柱   0：不封闭圆柱
cylinder_lines = 0; %1：显示分段线   0：只显示圆柱面
DrawCylinder3(plate1, plate2, radius, mesh_n,cylinder_color,cylinder_closed,cylinder_lines);  %画圆柱体目标

% ---------------
% task variables
% ---------------
ma = 6;     % dim of task a
mb = 3;     % dim of task b
mc = 6;     % dim of task c
sigma_tilde_a = zeros(ma,npti);
sigma_tilde_b = zeros(mb,npti);
sigma_tilde_c = zeros(mc,npti);

R_d = [0 1 0; 0 0 1; 1 0 0]; % desired end-effector orientation

eta2_quat_d = Rot2Quat([0 1 0; 1 0 0; 0 0 1]); % desired vehicle orientation 转向抓取
% eta2_quat_d = Rot2Quat([1 0 0; 0 1 0; 0 0 1]);

obj_pos  = [-0.3 0.36 1.192]';   % object position
obj_size = 0.2;             % object size
p_obst = [1.4 0 1.5]';    % obstacle position
ee_safety_dist = .2;      % end-effector safety distance from obstacle
W = diag([25 25 25 25 25 25 1*ones(1,n)]);invW=inv(W);

copy_DrawSnap(eta(:,1),[DH(:,1:3) q(:,1)],[DH_r(:,1:3) q_r(:,1)],PARAM,2.1);
axis auto
drawnow

%------------------------
% main loop - start
%------------------------
t_start_sim = clock();
for i=1:npti
    % current end-effector configuration
    T = DirectKinematics(eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
    eta_ee1(:,i)     = T(1:3,4);
    eta_ee_quat(:,i) = Rot2Quat(T(1:3,1:3));
    % desired end-effector trajectory
    eta_ee1_d(:,i) = trapezoidal2(eta_ee1(:,1),[-0.5 0 1.6]',[.3 .1 .6]',6,t(i)); % 末端坐标
    eta_ee_quat_d(:,i) = GenerateDesQuat(eta_ee_quat(:,1),R_d,7,12,t(i)); % orientation
%     eta_ee1_d(:,i) = traj(t(i));
    
    
    % controller
    if (KinOnly==1)
        % first-priority task - end-effector configuration
        sigma_tilde_a(:,i) = sigma_tilde05(eta_ee1_d(:,i),eta_ee_quat_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        J_a = J05(eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        pinvJ_a = mypinv(J_a,invW,'a');
        N_a = eye(6+n,6+n)-pinvJ_a*J_a;
        k_a = 10;
        % second-priority task - null roll/pitch
%         sigma_tilde_b(:,i) = sigma_tilde12([0 0]/180*pi,eta(4:6,i));
%         J_b = J12(eta(4:6,i),n);
%         pinvJ_b = mypinv(J_b,invW,'b');
%         k_b = 50;
       
        sigma_tilde_b(:,i) = sigma_tilde10(eta2_quat_d,eta(4:6,i));
        J_b = J10(6,eta(4:6,i));
        pinvJ_b = mypinv(J_b,invW,'b');
        k_b = 20;
        % third-priority task - robot nominal configuration (only second
        % joint at -45 deg
        J_ab = [J_a; J_b];
        pinvJ_ab = mypinv(J_ab,invW);
        N_ab = eye(6+n,6+n)-pinvJ_ab*J_ab;
        sigma_tilde_c(:,i) = sigma_tilde09(q(:,i),PARAM.q_nominal);
        J_c = J09(q(:,i));
        k_c = diag([80 80 80 0 0 0]);
        pinvJ_c = mypinv(J_c,invW,'c');

        %-------------------
        zita_d(:,i) =      pinvJ_a*k_a*sigma_tilde_a(:,i) + ...
                       N_a*pinvJ_b*k_b*sigma_tilde_b(:,i) + ...
                      N_ab*pinvJ_c*k_c*sigma_tilde_c(:,i);
        zita_d(:,i) = VectorSat(zita_d(:,i),PARAM.zita_limit);
    else
        tau(:,i) = zeros(6+n,1);
    end
    
    % do not modify remaing lines of the main loop
    % integration
    if i<npti
        [dzita(:,i), zita(:,i+1), eta(:,i+1), q(:,i+1)] = Integration(KinOnly, eta(:,i), [DH(:,1:3) q(:,i)],...
            zita(:,i), zita_d(:,i), tau(:,i), PARAM, Ts);
        q_r(1,i+1)   =  q(1,i+1);
        q_r(2:3,i+1) = -q(2:3,i+1);
    end
    EstimateEndSim(t_start_sim, i, npti);
     % UVMS三维轨迹
    X(i)=eta(1,i);Y(i)=eta(2,i);Z(i)=eta(3,i);

end
%------------------------
% main loop - end
%------------------------

figure
plate1 = [-1 0 1.6]; 
plate2 = [0.7 0 1.6];  %圆柱体两个端面中心点坐标
radius = 0.1;  %端面半径
mesh_n = 20; %精度
cylinder_color = [0.6 0.6 0.6];  % RGB三通道
cylinder_closed = 1; %1：封闭圆柱   0：不封闭圆柱
cylinder_lines = 0; %1：显示分段线   0：只显示圆柱面
DrawCylinder3(plate1, plate2, radius, mesh_n,cylinder_color,cylinder_closed,cylinder_lines);  %画圆柱体目标
plot3(X,Y,Z,'.b')
hold on
copy_DrawSnap(eta(:,npti),[DH(:,1:3) q(:,npti)],[DH_r(:,1:3) q(:,npti)],PARAM,2.1);
% plot3(eta_ee1_d(1,:),eta_ee1_d(2,:),eta_ee1_d(3,:),'g')
% plot3(eta_ee1(1,:),eta_ee1(2,:),eta_ee1(3,:),'k')
% if (exist('obj_pos','var')==1)
%     DrawCube(obj_pos,obj_size);
%     plot3([eta_ee1(1,npti) obj_pos(1)],[eta_ee1(2,npti) obj_pos(2)],[eta_ee1(3,npti) obj_pos(3)])
% end
axis auto


%------------------------
% graphics
%------------------------
if Graphics
%     figure
%     PlotEndEffector(t,eta_ee1_d,eta_ee1,eta_ee_quat_d,eta_ee_quat);
%     figure
%     PlotVelocities(t,zita);
%     figure
%     PlotConfiguration(t,eta,q);
%     figure
%     PlotTasks(t,sigma_tilde_a,sigma_tilde_b,sigma_tilde_c);


figure
plot(t,eta(1,:),'-b')
hold on
plot(t,eta(2,:),'-r')
hold on
plot(t,eta(3,:),'-k')
title('艇体位置')
xlabel('t [s]'),ylabel('位置 [m]')
grid on

figure
plot(t,eta(4,:)*180/pi,'-b')
hold on
plot(t,eta(5,:)*180/pi,'-r')
hold on
plot(t,eta(6,:)*180/pi,'-k')
hold on
title('艇体姿态')
xlabel('t [s]'),ylabel('姿态角 [deg]')
grid on

figure
plot(t,q(1,:)*180/pi,'-b')
hold on
plot(t,q(2,:)*180/pi,'-r')
hold on
plot(t,q(3,:)*180/pi,'-k')
hold on
title('关节角度')
xlabel('t [s]'),ylabel('角度 [deg]')
grid on

end

clear i t_start_sim
clear J Je T eta_dot


