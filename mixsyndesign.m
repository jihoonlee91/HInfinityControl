function [K,CL,gamma,info,G] = mixsyndesign(W1,W2,W3,eta)

global Glpv
G = Glpv.value('eta1',eta(1),'eta2',eta(2));
[K,CL,gamma,info] = mixsyn(G,W1,W2,W3);