function  dx = sys(t, x, A, B, K)
   u = K*x;
   dx = A*x + B*u;
end