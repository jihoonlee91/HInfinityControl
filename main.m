%% LPV modeling
close all
clear all
clc

addpath(genpath([pwd,'\LPVToolsV1.0']))

spec_data % load specification
aero_data % load aerodynamic data

V = 20;
h = 300;
eta1_grid = 0:0.5:1;
eta2_grid = 0:0.5:1;
n_eta1 = numel(eta1_grid);
n_eta2 = numel(eta2_grid);

% eta1 = pgrid('eta1',eta1_grid,[-0.5 0.5]); % rate-bounded
% eta2 = pgrid('eta2',eta2_grid,[-0.5 0.5]); % rate-bounded
eta1_pgrid = pgrid('eta1',eta1_grid); % rate-unbounded
eta2_pgrid = pgrid('eta2',eta2_grid); % rate-unbounded
eta_pgrid = [eta1_pgrid; eta2_pgrid];

tau_a = 1;

for n = 1:n_eta1
    for m = 1:n_eta2
        [Glon,alp_trim(n,m),delt_trim(n,m),dele_trim(n,m)] = lti_model(V,h,eta1_grid(n),eta2_grid(m));
        Zalpha = Glon.A(2,2)*V;
        Zdelta = Glon.B(2,2)*V;
        Malpha = Glon.A(3,2);
        Mdelta = Glon.B(3,2);
        Ap = [Zalpha/V,1;
            Malpha,0];
        Bp = [Zdelta/V;
            Mdelta];
        Cp = [1,0;
            0,1;
            Zalpha,0];
        Dp = [0;
            0;
            Zdelta];
        A = [Ap,Bp;zeros(1,2),-1/tau_a];
        B = [zeros(2,1);1/tau_a];
        C = [Cp,Dp;zeros(1,2),1];
        D = [zeros(3,1);0];
        Gsp(:,:,n,m) = ss(A,B,C,D); % short-period mode dynamics
    end
end

Domain = rgrid(eta1_pgrid,eta2_pgrid);
global Glpv
Glpv = pss(Gsp,Domain);

%%

%% H-infinity controller design
eta = [0.5; 0.5];
% W1 = makeweight(10,[1,0.1],0.01);
% W2 = makeweight(0.1,[32,0.32],1);
% W3 = makeweight(0.01,[1,0.1],10);
W1 = makeweight(10,[1,0.1],0.01);
W2 = [];
W3 = makeweight(0.01,[1,0.1],10);

[K,CL,gamma,info,G] = mixsyndesign(W1,W2,W3,eta);
gamma

%% Simulation
step(CL,30)