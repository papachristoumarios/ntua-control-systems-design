s = tf('s');
kp = 1000;
kd = 3;
G = zpk([], [0 -361.2], 4500);
C = pid(kp, 0, kd);
GCL = feedback(G*C, 1);
stepinfo(GCL)
step(GCL / s);
Kv = 4500 * 1000 / 361.2
ess = 1/Kv


figure;
rlocus(G);
figure;
rlocus(G*C);
