clear all; close all; clc;
lc = lines(7);
if ~ exist(fullfile(pwd,'images'),'dir'), mkdir images; end

lt = @(s) clipboard('copy',latex(s));

%% Modelling of the DC Motor

syms v_a T_e R L K_e K_t D_1 D_2 J_1 J_2 B

ia = sym('i_a_',[1,2]);
phi1 = sym('phi_1_',[1,2]);
w1 = sym('omega_1_',[1,2]);
phi2 = sym('phi_2_',[1,2]);
w2 = sym('omega_2_',[1,2]);
phi3 = sym('phi_3_',[1,2]);

x = [ia; phi1; w1; phi2; w2; phi3]
u = [v_a; T_e]


eqs = [       v_a == ia(1)*R + L*ia(2) + K_e*w1(1) ; 
        J_1*w1(2) == K_t*ia(1) - D_1*(phi1(1)-phi2(1)) ;
        J_2*w2(2) == D_1*(phi1(1)-phi2(1)) - D_2*(phi2(1)-phi3(1)) ;
               0 == D_2*(phi2(1)-phi3(1)) - B*phi3(2) + T_e ;
         phi1(2) == w1(1) ;
         phi2(2) == w2(1) ];
     
eqs = lhs(eqs) - rhs(eqs);
     
%% 1.a

[Am,Bm] = get_state_space(eqs, x(:,2), x(:,1), u)

lstring = sprintf('\t%s \n\t=\n \t%s \n\t*\n \t%s \n\t+\n \t%s \n\t*\n \t%s', ...
                  latex(x(:,2)), latex(Am), latex(x(:,1)), latex(Bm), latex(u));
clipboard('copy',lstring)

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

vars_sym   = [R, K_e, K_t, J_1,   J_2,   B,    D_1, D_2]; 
vars_value = [1, .1, .1, 1e-5, 4e-5, 2e-3, 20,  2];
repl = @(x) double(subs(x, vars_sym, vars_value));

Av = repl(Am)
Bv = repl(Bm)
Cv = repl(Cm)
Dv = repl(Dm)

double(eig(Av))


%% 1.e

Ce = [jacobian(x(:,1), [w1(1);w2(1)]).';
       repl(Cb) ];
De = [0*zeros(size(u.'));
       0*zeros(size(u.'));
       repl(Db)]

SS = ss(Av,Bv,double(Ce),double(De))

s = tf('s')
I = eye(size(Av));
G = tf((s*I-Av)\Bv);

tv  = linspace(0,0.08,1000);
uv = [10*ones(size(tv));
      0*ones(1,numel(tv)/2) -0.1*ones(1,numel(tv)/2) ];

% step(G(2,1)*10)

[ysol,tsol,xsol] = lsim(SS,uv,tv)

lw=2;

figure('Color','white')
subplot(5,1,1)
plot(tsol,uv(1,:), 'LineWidth', lw); grid on;
ylabel('v_a')
subplot(5,1,2)
plot(tsol,uv(2,:), 'LineWidth', lw); grid on;
ylabel('T_e')

subplot(5,1,3)
plot(tsol,ysol(:,1), 'LineWidth', lw); grid on;
ylabel('\omega_{1,1}')
subplot(5,1,4)
plot(tsol,ysol(:,2), 'LineWidth', lw); grid on;
ylabel('\omega_{2,1}')
subplot(5,1,5)
plot(tsol,ysol(:,3), 'LineWidth', lw); grid on;
ylabel('\omega_{3,1}')
xlabel('time [s]')


set(gca,'LooseInset',get(gca,'TightInset'))
saveas(gcf, fullfile(pwd,'images/1e_outputs'),'epsc')


%% 1.f

s = tf('s')
I = eye(size(Av));
G = Cv*inv(s*I-Av)*Bv + Dv
zpk(G(:,1))

tzero(G(:,1))

function [A,b] = get_state_space(eqs, xdot, x, u)
    A = -jacobian(eqs, xdot) \ jacobian(eqs, x);
    b = -jacobian(eqs, xdot) \ jacobian(eqs, u);
end

