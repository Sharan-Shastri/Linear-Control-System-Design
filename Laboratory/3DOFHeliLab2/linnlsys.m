function [A B] = stud_linnlsysc(x0,u0)
% Mass of front propeller assembly = motor + shield + propeller + body (kg)
M_f = 0.73; %1.15 / 2;
% Mass of back propeller assembly = motor + shield + propeller + body (kg)
M_b = 0.73; %1.15 / 2;
% M_b = 1.156 / 2;
% Mass of counter-weight (kg)
M_c = 1.87;
% Length of pendulum for the elevation axis (m)
Ld = 0.05;
% Distance between pitch pivot and each motor (m)
Lh = 7.0 * 0.0254;
% Distance between elevation pivot to helicopter body (m)
La = 26.0 * 0.0254;
% Distance between elevation pivot to counter-weight (m)
Lc = 18.5 * 0.0254;
% Distance from the pivot point to the helicopter body (m)
Le = 0.02;
% Gravitational Constant (m/s^2)
g = 9.81;  
% Moment of inertia about the elevation, pitch and travel axes (kg/m^2)
J_e = 0.86;
J_r = 0.044; %J_r = 0.044;
J_l = 0.82;
% Coefficient of viscous friction about the elevation, pitch and travel
% axes (kg*m^2/s)
n_e = 0.001*2;
n_r = 0.001*2;
n_l = 0.005*2;
% n_e = 0.05;
% n_r = 0.05;
% n_l = 0.1;
% Propeller force-thrust constant found experimentally (N/V)
K_m = 0.1188;
% K_m = 0.08;


d_a = atan((Ld+Le)/La);
d_c = atan(Ld/Lc);
d_h = atan(Le/Lh);

p(1) = (-(M_f+M_b)*g*La+M_c*g*Lc)/J_e;
p(2) = (-(M_f+M_b)*g*La*tan(d_a)+M_c*g*Lc*tan(d_c))/J_e;
p(3) = -n_e/J_e;
p(4) = K_m*La/J_e;
p(5) = (-M_f+M_b)*g*Lh/J_r;
p(6) = 0; %-(M_f+M_b)*g*Lh*tan(d_h)/J_r;
p(7) = -n_r/J_r;
p(8) = K_m*Lh/J_r;
p(9) = -n_l/J_l;
p(10) = -K_m*La/J_l;

% clear all

% syms p p1 p2 p3 p4 p5 p6 p7 p8 p9 p10

% p(1) = p1;p(2) = p2;p(3) = p3;p(4) = p4;p(5) = p5;p(6) = p6;p(7) = p7;p(8) = p8;p(9) = p9;p(10) = p10;

syms x1 x2 x3 x4 x5 x6 u1 u2 ddu

ddx(1,1) = x4;
ddx(2,1) = x5;
ddx(3,1) = x6;
ddx(4,1) = p(1)*cos(x1)+p(2)*sin(x1)+p(3)*x4;
ddx(5,1) = p(5)*cos(x2)+p(6)*sin(x2)+p(7)*x5;
ddx(6,1) = p(9)*x6;


ddu(1,1) = 0;
ddu(2,1) = 0;
ddu(3,1) = 0;
ddu(4,1) = u1*p(4)*cos(x2) +  u2*p(4)*cos(x2);              % OK
ddu(5,1) = p(8)*u1 - p(8)*u2;                               % OK
ddu(6,1) = u1*p(10)*sin(x2) + u2*p(10)*sin(x2);             % OK

if nargin < 2
    ddx_lin = solve(subs(ddx(4:6,:)+ddu(4:6,:),{'x1','x2','x3','x4','x5','x6'},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6)}));

    u0 = double([ddx_lin.u1,ddx_lin.u2])

end

%%


J_a = [diff(ddx+ddu,x1),diff(ddx+ddu,x2),diff(ddx+ddu,x3),diff(ddx+ddu,x4),diff(ddx+ddu,x5),diff(ddx+ddu,x6)];
J_b = [diff(ddx+ddu,u1),diff(ddx+ddu,u2)];



A = double(subs(J_a,{'x1','x2','x3','x4','x5','x6','u1','u2'},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),u0(1),u0(2)}));
B = double(subs(J_b,{'x1','x2','x3','x4','x5','x6','u1','u2'},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),u0(1),u0(2)}));






