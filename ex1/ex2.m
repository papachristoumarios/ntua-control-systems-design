kp = 13;
ki = 31.5;
C = pid(kp, ki, 0);
G = zpk([], [0 -361.2], 4500);
stepinfo(feedback(G*C, 1))
ess = 361.2 / (4500 * ki)
Kv = 1 / ess


figure;
rlocus(G);
figure;
rlocus(G*C);
