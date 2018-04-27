G = zpk([], [0 -3008 -400.26], 2.718e9);
kp = 0.37;
kd = 0.0008;
C = pid(kp, 0, kd);
stepinfo(feedback(G*C, 1))
Kv = 2.718e9 * kp / (400.26 * 3008)
ess = 1 / Kv

figure;
rlocus(G);
figure;
rlocus(G*C);
