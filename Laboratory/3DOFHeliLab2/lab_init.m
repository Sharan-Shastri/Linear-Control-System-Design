setup_lab_heli_3d_simple;


% Fetch linearized system
X0 = zeros(6,1);
[A,B] = linnlsys(X0);

% Define weighting matrices and compute K
Q_x = diag([1,1,1,0,0,0]);
Q_u = 0.1*eye(2);


K = lqr(A,B,Q_x,Q_u);


% If integral gain is present split up into K and K_I
% K should be 2x6 and K_I should be 2x2
K_I = zeros(2);

% Additional voltage, V_K is scalar (could be 2x1)
V_K = 0;

% Output vector is \eps and \lambda C is 2x6
C = [1,0,0,0,0,0;
     0,0,1,0,0,0];
K_r = inv(C*inv(-A+B*K)*B);