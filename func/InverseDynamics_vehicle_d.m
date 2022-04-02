function tau = InverseDynamics_vehicle_d(e, nu, dnu, rho, g0)
% Calculate Forces and moments acting on the vehicles
% Simulator Fossen ex. B.2
%
% tau = InverseDynamics_vehicle(e, nu, dnu, rho, g0);
%
% input:
%       e      dim 4x1     vehicle orientation (quaternions)
%       nu     dim 6x1     vehicle velocity
%       dnu    dim 6x1     vehicle acceleration
%       rho    dim 1x1     water density
%       g0     dim 3x1     gravity
%
% output:
%       tau	   dim 6x1     vehicle forces
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

u = nu(1);  
v = nu(2);  
w = nu(3);
p = nu(4);  
q = nu(5);  
r = nu(6);

L   = 5.3;                             
rv_g = [0 0 0.061];                   
rv_b = [0 0 -0.04];                    
m_v   = 5454.54/(rho/2*L^3);
if norm(g0)==0
    W = 0;
    B = 0;
else
    B = 2510;  
    W = 2230;
    
end
I_v = [2038, 13587, 13587, -13.58, -13.58, -13.58];
r2 = rho*L^2/2;
r3 = rho*L^3/2;
r4 = rho*L^4/2;
r5 = rho*L^5/2;
M_v = [6.019e+3  5.122e-8 -1.180e-2 -3.200e-5  3.325e+2 -6.731e-5
        5.122e-8  9.551e+3  3.717e-6 -3.802e+2 -3.067e-5 -4.736e+2
       -1.180e-2  3.717e-6  2.332e+4 -1.514e-4  2.683e+3 -4.750e-4
       -3.200e-5 -3.802e+2 -1.514e-4  4.129e+3  1.358e+1  8.467e+1
        3.325e+2 -3.067e-5  2.683e+3  1.358e+1  4.913e+4  1.357e+1
       -6.731e-5 -4.736e+2 -4.750e-4  8.467e+1  1.357e+1  2.069e+4];

xG = rv_g(1);  
yG = rv_g(2);    
zG = rv_g(3);
xB = rv_b(1);  
yB = rv_b(2);    
zB = rv_b(3);
Ix  = I_v(1);    
Iy  = I_v(2);     
Iz  = I_v(3);
Ixy = I_v(4);   
Iyz = I_v(5);    
Ixz = I_v(6);

% Parameters, hydrodynamic derivatives and main dimensions

Xpp   =  7.0e-3; Xqq    = -1.5e-2; Xrr   =  4.0e-3; Xpr   =  7.5e-4;
Xudot = -7.6e-3; Xwq    = -2.0e-1; Xvp   = -3.0e-3; Xvr   =  2.0e-2;
Xqds  =  2.5e-2; Xqdb2  = -1.3e-3; Xrdr  = -1.0e-3; Xvv   =  5.3e-2;
Xww   =  1.7e-1; Xvdr   =  1.7e-3; Xwds  =  4.6e-2; Xwdb2 =  0.5e-2;
Xdsds = -1.0e-2; Xdbdb2 = -4.0e-3; Xdrdr = -1.0e-2; Xqdsn =  2.0e-3;
Xwdsn =  3.5e-3; Xdsdsn = -1.6e-3;

Ypdot =  1.2e-4; Yrdot  =  1.2e-3; Ypq   =  4.0e-3; Yqr   = -6.5e-3;
Yvdot = -5.5e-2; Yp     =  3.0e-3; Yr    =  3.0e-2; Yvq   =  2.4e-2;
Ywp   =  2.3e-1; Ywr    = -1.9e-2; Yv    = -1.0e-1; Yvw   =  6.8e-2;
Ydr   =  2.7e-2;

Zqdot = -6.8e-3; Zpp    =  1.3e-4; Zpr   =  6.7e-3; Zrr   = -7.4e-3;
Zwdot = -2.4e-1; Zq     = -1.4e-1; Zvp   = -4.8e-2; Zvr   =  4.5e-2;
Zw    = -3.0e-1; Zvv    = -6.8e-2; Zds   = -7.3e-2; Zdb2  = -1.3e-2;
Zqn   = -2.9e-3; Zwn    = -5.1e-3; Zdsn  = -1.0e-2;

Kpdot = -1.0e-3; Krdot  = -3.4e-5; Kpq   = -6.9e-5; Kqr   =  1.7e-2;
Kvdot =  1.2e-4; Kp     = -1.1e-2; Kr    = -8.4e-4; Kvq   = -5.1e-3;
Kwp   = -1.3e-4; Kwr    =  1.4e-2; Kv    =  3.1e-3; Kvw   = -1.9e-1;
Kdb2  =  0;      Kpn    = -5.7e-4; Kprop =  0;

Mqdot = -1.7e-2; Mpp    =  5.3e-5; Mpr   =  5.0e-3; Mrr   =  2.9e-3;
Mwdot = -6.8e-3; Muq    = -6.8e-2; Mvp   =  1.2e-3; Mvr   =  1.7e-2;
Muw   =  1.0e-1; Mvv    = -2.6e-2; Mds   = -4.1e-2; Mdb2  =  3.5e-3;
Mqn   = -1.6e-3; Mwn    = -2.9e-3; Mdsn  = -5.2e-3;

Npdot = -3.4e-5; Nrdot  = -3.4e-3; Npq   = -2.1e-2; Nqr   =  2.7e-3;
Nvdot =  1.2e-3; Np     = -8.4e-4; Nr    = -1.6e-2; Nvq   = -1.0e-2;
Nwp   = -1.7e-2; Nwr    =  7.4e-3; Nv    = -7.4e-3; Nvw   = -2.7e-2;
Ndr   = -1.3e-2; Nprop  =  0;

% Drag forces and moments assuming block shaped body

dxL = L/10;
xL  = 0;
Ucf = sqrt((v+xL*r)^2+(w-xL*q)^2);
Cy = 0; Cz = 0; Cm = 0; Cn = 0;
if ~(Ucf == 0),
   for xL = 0:dxL:L
      Ucf = sqrt((v+xL*r)^2+(w-xL*q)^2);
      temp = (0.5*0.6*(v+xL*r)^2+0.6*(w-xL*q)^2)/Ucf;
      dCy = temp*(v+xL*r);
      Cy = Cy + dxL*dCy;
      dCz = temp*(w-xL*q);
      Cz = Cz + dxL*dCz;
      dCm = temp*(w+xL*q)*xL;
      Cm = Cm + dxL*dCm;
      dCn = temp*(v+xL*r)*xL;
      Cn = Cn + dxL*dCn;
   end
end

% Forces and moments without restoring forces

veic_f(1) = r3*((m_v+Xvr)*v*r + (Xwq-m_v)*w*q + Xvp*v*p) + ...
    r4*((m_v*xG/L+Xqq)*q^2 + (m_v*xG/L+Xrr)*r^2 - m_v*yG/L*p*q + ...
               (Xpr-m_v*zG/L)*p*r + Xpp*p^2 ) + ...
    r2*(Xvv*v^2 + Xww*w^2 );

veic_f(2) = r2*(Yv*u*v + Yvw*v*w) + ...
    r3*(Yp*u*p + Yr*u*r +Yvq*v*q + Ywp*w*p + Ywr*w*r) +...
    r4*(Ypq*p*q + Yqr*q*r) - ...
    m_v*(rho/2*L^3)*(u*r -w*p + xG*p*q - yG*(p^2+r^2) + zG*q*r)-rho/2*Cy;

veic_f(3) = r2*(Zw*w*u + Zvv*v^2) +...
    r3*(Zq*u*q + Zvp*v*p + Zvr*v*r) +...
    r4*(Zpp*p^2 + Zpr*p*r + Zrr*r^2) - ...
    m_v*(rho/2*L^3)*(v*p - u*q + xG*p*r + yG*q*r -zG*(p^2+q^2))+ rho/2*Cz;

veic_f(4) = r3*(Kv*u*v + Kvw*v*w) +...
    r4*(Kp*u*p + Kr*u*r + Kvq*v*q + Kwp*w*p + Kwr*w*r) +...
    r5*(Kpq*p*q + Kqr*q*r) +...
    (Iy-Iz)*q*r - Ixy*p*r - (r^2-q^2)*Iyz + Ixz*p*q - ...
    m_v*(rho/2*L^3)*(yG*(v*p-u*q) - zG*(u*r - w*p));

veic_f(5) = r3*(Muw*u*w + Mvv*v^2) +...
    r4*(Muq*u*q + Mvp*v*p + Mvr*v*r) +...
    r5*(Mpp*p^2 + Mpr*p*r + Mrr*r^2) + ...
    (Iz-Ix)*p*r + Ixy*q*r - Iyz*p*q - (p^2-r^2)*Ixz + ...
    m_v*(rho/2*L^3)*(xG*(v*p-u*q) - zG*(w*q-v*r)) - rho/2*Cm;

veic_f(6) = r3*(Nv*u*v + Nvw*v*w)+...
    r4*(Np*u*p + Nr*u*r + Nvq*v*q + Nwp*w*p + Nwr*w*r)+...
    r5*(Npq*p*q + Nqr*q*r) +...
    (Ix-Iy)*p*q + (p^2-q^2)*Ixy + Iyz*p*r - Ixz*q*r -...
    m_v*(rho/2*L^3)*(xG*(u*r-w*p) - yG*(w*q-v*r)) - rho/2*Cn ;

% restoring forces and moments
t1 = e(1)^2;
t2 = e(2)^2;
t3 = e(3)^2;
%t4 = e(1)*e(2);
t5 = e(1)*e(3);
t6 = e(2)*e(3);
t7 = e(4)*e(1);
t8 = e(4)*e(2);
%t9 = e(4)*e(3);
t10 = e(4)^2;
temp1 = - t10 + t1 + t2 -t3;
temp2 = 2*(t8 - t5);
temp3 = 2*(t7 + t6);

veic_f(1) = veic_f(1) - temp2*(W - B);
veic_f(2) = veic_f(2) + temp3*(W - B);
veic_f(3) = veic_f(3) - temp1*(W - B);


veic_f(4) = veic_f(4) - temp1*(yG*W - yB*B) - temp3*(zG*W - zB*B);
veic_f(5) = veic_f(5) + temp1*(xG*W - xB*B) - temp2*(zG*W - zB*B);
veic_f(6) = veic_f(6) + temp3*(xG*W - xB*B) + temp2*(yG*W - yB*B);

tau = M_v*dnu - veic_f';

