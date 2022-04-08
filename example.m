s = zpk('s');
G = (s-1)/(s+1)^2;

[num,den] = tfdata(G);
[A,B,C,D] = tf2ss([0 1 -1],[1 2 1]);
G = ss(A,B,C,D);

W1 = makeweight(10,[1 0.1],0.01);
W2 = makeweight(0.1,[32 0.32],1);
W3 = makeweight(0.01,[1 0.1],10);

bodemag(W1,W2,W3)

[K,CL,gamma] = mixsyn(G,W1,W2,W3);
gamma

S = feedback(1,G*K);
KS = K*S;
T = 1-S;
sigma(S,'b',KS,'r',T,'g',gamma/W1,'b-.',ss(gamma/W2),'r-.',gamma/W3,'g-.',{1e-3,1e3})
legend('S','KS','T','GAM/W1','GAM/W2','GAM/W3','Location','SouthWest')
grid

step(CL)