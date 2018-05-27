s = tf('s');
kp = 11.7;
ki = 0.55;
C = pid(kp, ki, 0);
figure;
G = zpk([], [0 -361.2], 4500);
GCL = feedback(G*C, 1);
step(GCL / s);
stepinfo(GCL)
ess = 361.2 / (4500 * ki)
Kv = 1 / ess


figure;
rlocus(G);
figure;
rlocus(G*C);
