%% 3-DOF helicopter nonlinear model parameters %%

Mf = 0.71; %[kg] Mass of front propeller assembly
Mb = 0.71; %[kg] Mass of back propeller assembly
Mc = 1.69; %[kg] Mass of the counterwight
Ld = 0.05; %[m] Length of pendulum for elevention axis
Lc = 0.44; %[m] Distance from pivot point to counterweight
La = 0.62; %[m] Distance from pivot point to helicopter body
Le = 0.02; %[m] Length of pendulum for pitch axis
Lh = 0.18; %[m] Distance from pitch axis to motor
g = 9.81; %[m/s^2] Gravitational acceleration
Km = 0.12; %[N/V] Propeller force-thrust constant
Jeps = 0.86; %[kg*m^2] Moment of inertia level axis
Jro = 0.044; %[kg*m^2] Moment of inertia pitch axis
Jlam = 0.82; %[kg*m^2] Moment of inertia travel axis
Neps = 0.001; %[kg*m^2/s] Viscous friction level axis
Nro = 0.001; %[kg*m^2/s] Viscous friction pitch axis
Nlam = 0.005; %[kg*m^2/s] Viscous friction travel axis

% Coefficients p
p1 = (-(Mf+Mb)*g*La+Mc*g*Lc)/Jeps;
p2 = (-(Mf+Mb)*g*(Ld+Le)+Mc*g*Ld)/Jeps;
p3 = -Neps/Jeps;
p4 = Km*La/Jeps;
p5 = (-Mf+Mb)*g*Lh/Jro;
p6 = -(Mf+Mb)*g*Le/Jro;
p7 = -Nro/Jro;
p8 = Km*Lh/Jro;
p9 = -Nlam/Jlam;
p10 = -Km*La/Jlam;

% initial conditions
eps0 = 0;
ro0 = 0;
lam0 = 0;
eps0dot = 0;
ro0dot = 0;
lam0dot = 0;