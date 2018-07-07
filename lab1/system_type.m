s = tf('s'); 

G1 = zpk([], [-1 -1 -1], 1);
G2 = zpk([], [-1 -1 0], 1);
G3 = zpk([], [-1 0 0], 1);

figure;
subplot(1, 3, 1);
rlocus(G1);
subplot(1, 3, 2);
rlocus(G2);
subplot(1, 3, 3);
rlocus(G3);
suptitle('Root loci for Type-3, Type-2 and Type-1 Systems');

G1CL = feedback(G1, 1);
G2CL = feedback(G2, 1);

% closed loop is unstable forall K > 0
G3CL = feedback(G3, 1);


figure;
subplot(1, 2, 1);
step(G1CL); 

subplot(1, 2, 2);
step(G2CL / s); 
title('Ramp response');

suptitle('Step and ramp response for Type-0 and Type-1 System');


