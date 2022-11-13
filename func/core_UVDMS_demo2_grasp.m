%
% HUST-UVDMS
%
% Yuhao Liu   2019/12/30
% 
% Main simulator for PID control of underwater grab operation based on HUST-UVDMS dynamic model

%------------------------
% simulation parameters 
%------------------------
t_f = 30;       % final simulation time [s]
Ts  = 0.005;	% sampling time [s]
t   = 0:Ts:t_f;	% time vector [s]
npti = length(t);

KinOnly   = 0;  % kinematic simulati on only, controller needs to provide reference velocities
                % if null the simulation is dynamic and the controller
                % outputs the forces/moments/torques
Graphics  = 1;	% provide or not graphics in the end of the simulation

% Movie     = 1;  % generate movie for process of simulation

% Definition of the variables
n   = 2*size(DH,1);
eta           = zeros(6,npti); 
q             = zeros(n,npti);
zita          = zeros(6+n,npti); 
eta_d         = zeros(6,npti); 
q_d           = zeros(n,npti);
zita_d        = zeros(6+n,npti);
gamma         = zeros(9,npti);
gamma(1,1)    = 0;
gamma(6,1)    = 0;
gamma_q       = zeros(n,npti);
gamma_v       = zeros(6,npti);
dzita         = zeros(6+n,npti);
tau           = zeros(6+n,npti);
q_draw        = zeros(6,301);
TS            = 0.1;
t_draw        = 0:TS:t_f;


DrawSnap(eta(:,1),[DH(:,1:3) q(1:3,1)],[DH_r(:,1:3) q(4:6,1)],PARAM,2.1,0);
axis auto
drawnow

DrawCylinder3([-0.3  0.8 1.192], [-0.3 -0.8 1.192], 0.1, 20,[0.6 0.6 0.6],1,0);

%------------------------
% main loop - start
%------------------------
t_start_sim = clock();
for i=1:npti
    
    % controller
    if (KinOnly==1)
        fprintf('\n ------------------------------------------------------------------- ');
        fprintf('\n ERROR in core_simulator_demo2.m: this demo only runs with KinOnly=0 ');
        fprintf('\n ------------------------------------------------------------------- ');
        return;
    else
        zita_d(:,i) = 0*zita_d(:,i);
        eta_d(:,i)  = 0*eta_d(:,i);
        if i<npti
            % joint planning
            % step1: spread manipulator from vehicle
            if (0<i) && (i<=800)
                ib1 = 0;
                q_d(1,i+1) = threePoly(0, 0*pi/180, i, ib1, Ts);
                q_d(2,i+1) = threePoly(0, 40*pi/180, i, ib1, Ts);
                q_d(3,i+1) = threePoly(0, 0, i, ib1, Ts);
                q_d(4,i+1) = threePoly(0, 0*pi/180, i, ib1, Ts);
                q_d(5,i+1) = threePoly(0, -40*pi/180, i, ib1, Ts);
                q_d(6,i+1) = threePoly(0, 0, i, ib1, Ts);
            end
            if (800<i) && (i<=900)
                ib2 = 800;
                q_d(1,i+1) = 0*pi/180;
                q_d(2,i+1) = 40*pi/180;
                q_d(3,i+1) = 0;
                q_d(4,i+1) = 0;
                q_d(5,i+1) = -40*pi/180;
                q_d(6,i+1) = 0;
            end
            % step2: put manipulator down
            if (900<i) && (i<=2700)
                ib3 = 900;
                q_d(1,i+1) = threePoly(0, 90*pi/180, i, ib3, Ts);
                q_d(2,i+1) = 40*pi/180;
                q_d(3,i+1) = threePoly(0, -40*pi/180, i, ib3, Ts);
                q_d(4,i+1) = threePoly(0, 90*pi/180, i, ib3, Ts);
                q_d(5,i+1) = -40*pi/180;
                q_d(6,i+1) = threePoly(0, 40*pi/180, i, ib3, Ts);
            end
            if (2700<i) && (i<=2800)
                ib4 = 2700;
                q_d(1,i+1) = 90*pi/180;
                q_d(2,i+1) = 40*pi/180;
                q_d(3,i+1) = -40*pi/180;
                q_d(4,i+1) = 90*pi/180;
                q_d(5,i+1) = -40*pi/180;
                q_d(6,i+1) = 40*pi/180;
            end
            % step3: grab
            if (2800<i) && (i<=3600)
                ib5 = 2800;
                q_d(1,i+1) = 90*pi/180;
                q_d(2,i+1) = threePoly(40*pi/180, 0*pi/180, i, ib5, Ts);
                q_d(3,i+1) = threePoly(-40*pi/180, 0*pi/180, i, ib5, Ts);
                q_d(4,i+1) =  90*pi/180;
                q_d(5,i+1) = threePoly(-40*pi/180, 0*pi/180, i, ib5, Ts);
                q_d(6,i+1) = threePoly(40*pi/180, 0*pi/180, i, ib5, Ts);
            end
            if 3600<i
                ib6 = 2800;
                q_d(1,i+1) = 90*pi/180;
                q_d(2,i+1) = 0*pi/180;
                q_d(3,i+1) = 0*pi/180;
                q_d(4,i+1) = 90*pi/180;
                q_d(5,i+1) = 0*pi/180;
                q_d(6,i+1) = 0*pi/180;
            end
        end
        
        % "basic" controller, vehicle is implementing
        % an adaptive action ignoring the presence of the manipulator
        % then, PID at joints
        % vehicle controller
        Kdl = 2*diag([3000 3000 3000 300 2500 900]);
        lambda = diag([.08 .08 .08 1 1 1]);
        invKtheta = diag([4 4 4 4 4 4 4 4 4]);

        e   = Rpy2Quat(eta(4:6,i));
        e_d = Rpy2Quat(eta_d(4:6,i));
        e_o = e(4)*e_d(1:3) - e_d(4)*e(1:3) + cross(e_d(1:3),e(1:3));
        eta_tilde_1 = eta_d(1:3,i) - eta(1:3,i);
        nu_tilde = zita_d(1:6,i) - zita(1:6,i); 
        R = Quat2Rot(e);
        s = lambda*[R*eta_tilde_1;  e_o] + nu_tilde;
        phi_p = [zeros(3,3)    R'            zeros(3,3);
                 S(R'*[0;0;1])  zeros(3,3)   R'];
%         tau(1:6,i) = Kd*s + phi_p*gamma(1:9,i);

        % PID at vehicle
        Kp1 = diag([50000 50000 50000]);
        Ki1 = diag([1 1 1]);
        Kd1 = diag([50000 50000 50000]);
        Kp2 = diag([100000 100000 100000]);
        Ki2 = diag([2 2 2]);
        Kd2 = diag([50000 50000 50000]);

        if i<=2000
            tau(1:6,i)=zeros(6,1);
        else
            tau(1:3,i) =  Kp1*(eta_d(1:3,i)-eta(1:3,i)) + Kd1*(zita_d(1:3,i)-zita(1:3,i))+Ki1*gamma_v(1:3,i);
            tau(4:6,i) =  Kp2*(eta_d(4:6,i)-eta(4:6,i)) + Kd2*(zita_d(4:6,i)-zita(4:6,i))+Ki2*gamma_v(4:6,i);
        end
        if 2000<i && i<npti
            gamma_v(1:3,i+1) = gamma_v(1:3,i) + Ki1*(eta_d(1:3,i)-eta(1:3,i));
            gamma_v(4:6,i+1) = gamma_v(4:6,i) + Ki2*(eta_d(4:6,i)-eta(4:6,i));
        end
        
%         tau(1:6,i) = 0*tau(1:6,i);
        if i<npti
            gamma(1:9,i+1) = gamma(1:9,i) + invKtheta*phi_p'*s;
        end
        % PID at joints
        Kpl = diag([5000 2000 2000]);
        Kil = diag([0.1 0.5 0.5]);
        Kdl = diag([1300 700 700]);
        Kpr = diag([5000 2000 2000]);
        Kir = diag([0.1 0.5 0.5]);
        Kdr = diag([1300 700 700]);
        
        tau(7:9,i) = Kpl*(q_d(1:3,i)-q(1:3,i)) + Kdl*(zita_d(7:9,i)-zita(7:9,i)) + Kil*gamma_q(1:3,i);
        tau(10:12,i) = Kpr*(q_d(4:6,i)-q(4:6,i)) + Kdr*(zita_d(10:12,i)-zita(10:12,i)) + Kir*gamma_q(4:6,i);
        if i<npti
            gamma_q(1:3,i+1) = gamma_q(1:3,i) + Kil*(q_d(1:3,i)-q(1:3,i));
            gamma_q(4:6,i+1) = gamma_q(4:6,i) + Kir*(q_d(4:6,i)-q(4:6,i));
        end
    end

    % do not modify remaing lines of the main loop ,
    % integration
    if i<npti
        [dzita(:,i), zita(:,i+1), eta(:,i+1), q(:,i+1)] = Integration(KinOnly, eta(:,i), [DH(:,1:3) q(1:3,i)],[DH(:,1:3) q(4:6,i)], ...
            zita(:,i), zita_d(:,i), tau(:,i), PARAM, Ts);
    end
    EstimateEndSim(t_start_sim, i, npti);
    
end
%------------------------
% main loop - end
%------------------------

figure
DrawCylinder3([-0.3  0.8 1.192], [-0.3 -0.8 1.192], 0.1, 20,[0.6 0.6 0.6],1,0);
DrawSnap(eta(:,npti),[DH(:,1:3) q(1:3,npti)],[DH_r(:,1:3) q(4:6,npti)],PARAM,2.1,0);
% robot traj
plot3(eta(1,:),eta(2,:),eta(3,:),'.r');
axis auto


%------------------------
% graphics
%------------------------
if Graphics
    figure
    PlotForces(t,tau);
    figure
    PlotAccelerations(t,dzita);
    figure
    PlotVelocities(t,zita);
    figure
    PlotConfiguration(t,eta,q);
end


clear i t_start_sim
clear J Je T eta_dot
clear eta_tilde_1 phi_p


