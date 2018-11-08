clear all;
close all;
clc;


syms va Te R L Ke Kt D1 D2 J1 J2 B

ia = sym('i_a_',[1,2]);
phi1 = sym('phi_1_',[1,2]);
w1 = sym('omega_1_',[1,2]);
phi2 = sym('phi_2_',[1,2]);
w2 = sym('omega_2_',[1,2]);
phi3 = sym('phi_3_',[1,2]);

x = [ia; phi1; w1; phi2; w2; phi3]
u = [va; Te]


eqs = [       va == ia(1)*R + L*ia(2) + Ke*w1(1) ; 
        J1*w1(2) == Kt*ia(1) - D1*(phi1(1)-phi2(1)) ;
        J2*w2(2) == D1*(phi1(1)-phi2(1)) - D2*(phi2(1)-phi3(1)) ;
               0 == D2*(phi2(1)-phi3(1)) - B*phi3(2) + Te ;
         phi1(2) == w1(1) ;
         phi2(2) == w2(1) ];
     
eqs = lhs(eqs) - rhs(eqs);
     
%% 1.a

[Am,Bm] = get_state_space(eqs, x(:,2), x(:,1), u)

%% 1.b

x = x(2:end,:)

eqs_s = subs(eqs, ia(2), 0);
ia_v = solve(eqs_s(1), ia(1));
eqs_s = subs(eqs_s, ia(1), ia_v);
eqs_s = eqs_s(2:end);

[Am,Bm] = get_state_space(eqs_s, x(:,2), x(:,1), u)


%% 1.c

% Case 1
x(:,1)
Cm = jacobian(x(:,1), [phi2(1);w2(1)]).'
Dm = 0;

% Case 2
[Ca,Da] = get_state_space( subs(eqs(1),ia(2),0), ia(1), x(:,1), u);
[Cb,Db] = get_state_space( eqs(4)              , phi3(2), x(:,1), u);

Cm = [Ca;Cb]
Dm = [Da;Db]


%% 1.d

vars_sym   = [R, Ke, Kt, J1,   J2,   B,    D1, D2]; 
vars_value = [1, .1, .1, 1e-5, 4e-5, 2e-3, 20,  2];
repl = @(x) double(subs(x, vars_sym, vars_value));

Av = repl(Am)
Bv = repl(Bm)
Cv = repl(Cm)
Dv = repl(Dm)

double(eig(Av))


%% 1.e

Cv2 = [jacobian(x(:,1), [w1(1);w2(1)]).';
       repl(Cb) ];
Dv2 = [0*zeros(size(u.'));
       0*zeros(size(u.'));
       repl(Db)]

SS = ss(Av,Bv,double(Cv2),double(Dv2))

s = tf('s')
I = eye(size(Av));
G = tf((s*I-Av)\Bv);

tv  = 0:0.0001:0.0801;
uv = [10*ones(size(tv));
     0*ones(1,numel(tv)/2) -0.1*ones(1,numel(tv)/2) ];

% step(G(2,1)*10)

lsim(SS,uv,tv)


%% 1.f

s = tf('s')
I = eye(size(Av));
G = Cv*inv(s*I-Av)*Bv + Dv

figure
pzmap(minreal(G(1,1),1e-3))
figure
pzmap(minreal(G(2,1),1e-3))


fprinf('G_()')
minreal(G(:,1),1e-3)

figure
bode(minreal(G(:,1),1e-3))
figure
nyquist(minreal(G(:,1),1e-3))

function [A,b] = get_state_space(eqs, xdot, x, u)
    A = -jacobian(eqs, xdot) \ jacobian(eqs, x);
    b = -jacobian(eqs, xdot) \ jacobian(eqs, u);
end



