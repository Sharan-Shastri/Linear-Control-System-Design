% Oskar's test script
setup_lab_heli_3d_various_controler;
o_nonlinear_model;
A = A_lin;
B = B_lin;


%%
% Without integrator
C = [[1,0,0;0,0,1],zeros(2,3)];
Q = diag([100 0 10 25 0 0]);
R = 0.1*diag([1 1]);    
L = lqr( A, B, Q, R );
[U_svd S_svd V_svd]= svd(C*inv(A-B*L)*B,0);
Kr = -V_svd*inv(S_svd)*U_svd';
disp( ' ' )
disp( 'Calculated LQ controller gain elements: ' )
L
Kr
Li = zeros(2,3)

V_k = [10.83,10.83];
V_k = [0,0];
   

%%
% With integrator
C = [[1,0,0;0,0,1],zeros(2,3)];

Ae = [A zeros(6,size(C,1));...
     -C zeros(size(C,1),size(C,1))];
Be = [B;zeros(size(C,1),2)];

Q = diag([50 0 10 50 0 100, 30, 0.5]);
R = 0.1*diag([1 1]);    
L = lqr( Ae, Be, Q, R );
Li = L(:,end-size(C,1)+1:end);
L(:,end-size(C,1)+1:end) = [];

L
Li
Kr = [L(:,1),L(:,3)]


%%

% Speed control
C = [1,0,0,0,0; 0,0,0,0,1];

As = A; As(3,:) = []; As(:,3) = [];
Bs = B; Bs(3,:) = [];

Ae = [As zeros(5,size(C,1));...
     -C zeros(size(C,1),size(C,1))];
Be = [Bs;zeros(size(C,1),2)];

Q = diag([50 0 0 0 100, 30, 1]);
R = 0.1*diag([1 1]);    
L = lqr( Ae, Be, Q, R );
Li = L(:,end-size(C,1)+1:end);
L(:,end-size(C,1)+1:end) = [];

F = eye(6); F(3,:) = [];


% L = [L(:,1:2),[0;0],L(:,3:5)]
Li
Kr = [L(:,1),L(:,5)]
