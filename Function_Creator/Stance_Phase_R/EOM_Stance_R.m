%% script SymbolicComputationOfEQM

% Author: Yu-Ming Chen, University of Michigan, Ann Arbor
% Email: yminchen@umich.edu
% Date: 01/14/2017 

% This code is modified from an example "SymbolicComputationofEOM.m"
% shared by Nils Smit-Anseeuw, University of Michigan, Ann Arbor.

% This MATLAB script performs the symbolic computation of the individual
% components of the equations of motion (Mass Matrix, Differentiable
% Forces, etc.).  The resulting functions are saved as automatically
% generated mat-files in the directory 'AutoGeneratedFcts'.  

%% Definitions
clear
% Generalized coordinates
syms s1 phi1 theta s2 phi2
q    = [s1 phi1 theta s2 phi2];
% Generalized speeds
syms ds1 dphi1 dtheta ds2 dphi2
dqdt = [ds1 dphi1 dtheta ds2 dphi2];

% Define the necessary parameter subset:
% constant position of m1
syms x1 y1
% Gravity
syms g
% Spring stiffness 
syms k1 k2
% Segment dimensions:
syms L_sp0 L_mB     % spring original length and body length 
% Masses/Inertia:
syms mB IB
syms m2

param = [x1 y1 g k1 k2 L_sp0 L_mB mB IB m2];


%% DYNAMICS (obtained via the Euler-Lagrange equation)

% CoG-positions (from kinematics):
CoG_m1 = [x1;
          y1];
CoG_mB = [x1-s1*sin(phi1);
          s1*cos(phi1)];
CoG_m2 = [CoG_mB(1) + s2*sin(phi1-theta+phi2);
          CoG_mB(2) - s2*cos(phi1-theta+phi2)];
         
% CoG-velocities (computed via jacobians):
d_CoG_mB = jacobian(CoG_mB,q)*dqdt.';
d_CoG_m2 = jacobian(CoG_m2,q)*dqdt.';

% Potential Energy (due to gravity):
V = CoG_mB(2)*mB*g + 0.5*k1*(s1-L_sp0)^2 + CoG_m2(2)*m2*g + 0.5*k2*(s2-L_sp0)^2;
V = simplify(V);

% Kinetic Energy:         
T = 0.5 * (mB * sum(d_CoG_mB.^2) + ...
           IB * (dphi1-dtheta)^2 + ...
           m2 * sum(d_CoG_m2.^2) );
T = simplify(T);

% Lagrangian:
L = T-V;
% Partial derivatives:
dLdq   = jacobian(L,q).';
dLdqdt = jacobian(L,dqdt).';
      
% Compute Mass Matrix and its inverse:
M = jacobian(dLdqdt,dqdt);
M = simplify(M);
invM = inv(M);
invM = simplify(invM);

% Compute the coriolis and gravitational forces:
dL_dqdt_dt = jacobian(dLdqdt,q)*dqdt.';
f_cg = dLdq - dL_dqdt_dt;
f_cg = simplify(f_cg);

% The equations of motion are given with these functions as:   
% M * dqddt = f_cg(q, dqdt) + u;

%% KINEMATICS (for collision detection and graphical output)
% Two ends of body:
BodyL = [CoG_mB(1) - L_mB*cos(phi1-theta);
         CoG_mB(2) - L_mB*sin(phi1-theta)];
BodyR = [CoG_mB(1) + L_mB*cos(phi1-theta);
         CoG_mB(2) + L_mB*sin(phi1-theta)];
             
% Jacobian of two ends of body:
JBodyL = jacobian(BodyL,q);
JBodyL = simplify(JBodyL);
JBodyR = jacobian(BodyR,q);
JBodyR = simplify(JBodyR);

% CoGs (incl orientation of the segments) use for animations:
CoGs = [CoG_m1, CoG_mB,     CoG_m2;
        0,      phi1-theta, 0];
    
% Links use for animations
links = [BodyL, BodyR, CoG_mB, CoG_m1, CoG_m2];

%% Others
% Energy
E = T+V;
E = simplify(E);

%% Create MATLAB-functions:
if ~exist('AutoGeneratedFcts','dir')
    mkdir('AutoGeneratedFcts')
end

matlabFunction(M,'file','AutoGeneratedFcts\MassMatrix','vars',[q, param]);
matlabFunction(invM,'file','AutoGeneratedFcts\InvMassMatrix','vars',[q, param]);
matlabFunction(f_cg,'file','AutoGeneratedFcts\F_CoriGrav','vars',[q, dqdt, param]);
matlabFunction(E,'file','AutoGeneratedFcts\Energy','vars',[q, dqdt, param]);

matlabFunction(BodyL,'file','AutoGeneratedFcts\BodyL','vars',[q, param]);
matlabFunction(BodyR,'file','AutoGeneratedFcts\BodyR','vars',[q, param]);
matlabFunction(JBodyL,'file','AutoGeneratedFcts\ContactJacobianL','vars',[q, param]);
matlabFunction(JBodyR,'file','AutoGeneratedFcts\ContactJacobianR','vars',[q, param]);
matlabFunction(CoGs,'file','AutoGeneratedFcts\CoGPositions','vars',[q, param]);
matlabFunction(links,'file','AutoGeneratedFcts\LinkPositions','vars',[q, param]);