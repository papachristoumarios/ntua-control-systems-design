
% system
A = [0 1 0 0; 20.6 0 0 0; 0 0 0 1; -0.5 0 0 0];
B = [0; -1; 0; 0.5];
C = [1 0 0 0; 0 0 1 0];
D = [0 ; 0];
x0 = [-0.2; -0.06; 0.01; 0.3];
zeta = 0.5;
ts = 1.5;
omega_n = 4 / (zeta * ts);
t = 0:0.01:10;
r = 0.2*ones(size(t));

states = {'theta' 'theta_dot' 'x' 'x_dot'};
inputs = {'u'};
outputs = {'theta'; 'x'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);


% controllability
C_matrix = ctrb(A, B);
rank(C_matrix)

% A Pole Placement
omega_d = omega_n * sqrt(1 - zeta^2);
sigma = - zeta * omega_n;
p = sigma + omega_d * 1i;

poles = [p conj(p) -3000 -3500];
Kp = place(A, B, poles);

sys_cl_sf = ss(A - B * Kp, B, C, D,'statename',states,'inputname',inputs,'outputname',outputs);

figure;
[y_sf,t,x_sf]=lsim(sys_cl_sf,r,t);
plot(t, x_sf);
legend('theta', 'theta_{dot}', 'x', 'x_{dot}');
title('State feedback control with pole placement');
xlabel('Time (sec)');



% B LQR 
Q = eye(4);
R = 1;
[K, S, e] = lqr(A,B,Q,R);
N = 0;

newA = A - B * K;

states = {'theta' 'theta_dot' 'x' 'x_dot'};
inputs = {'r'};
outputs = {'theta'; 'x'};

sys_cl_lqr = ss(newA, B, C, D,'statename',states,'inputname',inputs,'outputname',outputs);

figure;
[y,t,x_lqr]=lsim(sys_cl_lqr,r,t);
plot(t, x_lqr);
legend('theta', 'theta_{dot}', 'x', 'x_{dot}');
xlabel('Time (sec)');
title('LQR');
 
% C Send the system to desired position
xr = [0 0 1 0]';
xf = inv(A - B * Kp) * B * Kp * xr;
u = -Kp * x_sf' - Kp * xr;
figure;
plot(t, u);
title('Input u(t) to send system to desired position');
xlabel('Time (sec)');
ylabel('u(t)');

x_new = x_sf;
[rown, coln] = size(x_sf);
for j = 1 : rown
    x_new(j, :) = x_new(j, :) + xf';
end;
figure;
plot(t, x_new);
title('State feedback sending position to desired position');
legend('theta', 'theta_{dot}', 'x', 'x_{dot}');

velocity = x_new(:, 3);
acceleration = x_new(:, 4);
mass = 1;
power = mass * acceleration .* velocity;

figure;
plot(t, power);
title('Power Assuming mass = 1kg');
xlabel('Time (sec)');



% D Luenberger observer
O_matrix = obsv(A, C);
rank(O_matrix);

% State Feedback
L = place(A', C', poles)';
A_ = [(A-B*Kp) (B*Kp); zeros(size(A)) (A-L*C)];
B_ = [B; zeros(size(B))];
C_ = [C zeros(size(C))];
D_ = [0; 0];

states = {'theta' 'theta_dot' 'x' 'x_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'r'};

sys_est_cl = ss(A_,B_,C_,D_,'statename',states,'inputname',inputs,'outputname',outputs);

figure;
[y,t,x]=lsim(sys_est_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
title('Luenberger Observer Control')
legend('theta', 'x');

% Luenberger with LQR 
[Llqr, S, e] = lqr(A',C',Q,eye(2));
Llqr = Llqr';
A__ = [(A-B*K) (B*K); zeros(size(A)) (A-Llqr*C)];

states = {'theta' 'theta_dot' 'x' 'x_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'r'};

sys_est_cl = ss(A__,B_,C_,D_,'statename',states,'inputname',inputs,'outputname',outputs);

figure;
[y,t,x]=lsim(sys_est_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
title('Luenberger Observer Control with LQR')
legend('theta', 'x');


