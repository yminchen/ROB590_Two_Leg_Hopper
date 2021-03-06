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
syms xB yB theta s1 phi1 s2 phi2
q    = [xB yB theta s1 phi1 s2 phi2];
% Generalized speeds
syms dxB dyB dtheta ds1 dphi1 ds2 dphi2
dqdt = [dxB dyB dtheta ds1 dphi1 ds2 dphi2];


% Define the necessary parameter subset:
% Gravity
syms g
% Spring stiffness 
syms k1 k2 
% Segment dimensions:
syms L_sp0 L_mB     % spring original length and body length 
% Masses/Inertia:
syms mB IB
syms m1 m2

param = [g k1 k2 L_sp0 L_mB mB IB m1 m2];


%% DYNAMICS (obtained via the Euler-Lagrange equation)

% CoG-positions (from kinematics):
CoG_mB = [xB;
          yB];
CoG_m1 = [xB + s1*sin(theta+phi1);
          yB - s1*cos(theta+phi1)];
CoG_m2 = [xB + s2*sin(theta+phi2);
          yB - s2*cos(theta+phi2)];
         
% CoG-velocities (computed via jacobians):
d_CoG_mB = jacobian(CoG_mB,q)*dqdt.';
d_CoG_m1 = jacobian(CoG_m1,q)*dqdt.';
d_CoG_m2 = jacobian(CoG_m2,q)*dqdt.';

% Potential Energy (due to gravity):
V = CoG_mB(2)*mB*g + CoG_m1(2)*m1*g + 0.5*k1*(s1-L_sp0)^2 + CoG_m2(2)*m2*g + 0.5*k2*(s2-L_sp0)^2;
V = simplify(V);

% Kinetic Energy:         
T = 0.5 * (mB * sum(d_CoG_mB.^2) + ...
           IB * dtheta^2 + ...
           m1 * sum(d_CoG_m1.^2) + ...
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
BodyL = [xB - L_mB/2*cos(theta);
         yB - L_mB/2*sin(theta)];
BodyR = [xB + L_mB/2*cos(theta);
         yB + L_mB/2*sin(theta)];
             
% Jacobian of two ends of body:
JBodyL = jacobian(BodyL,q);
JBodyL = simplify(JBodyL);
JBodyR = jacobian(BodyR,q);
JBodyR = simplify(JBodyR);

% CoGs (incl orientation of the segments) use for animations:
CoGs = [CoG_mB, CoG_m1, CoG_m2;
        theta , 0,      0];

% Links use for animations
links = [BodyL, BodyR, CoG_mB, CoG_m1, CoG_m2];
links = simplify(links);

%% Create MATLAB-functions:
if ~exist('AutoGeneratedFcts','dir')
    mkdir('AutoGeneratedFcts')
end

matlabFunction(M,'file','AutoGeneratedFcts\MassMatrix','vars',[q, param]);
matlabFunction(invM,'file','AutoGeneratedFcts\InvMassMatrix','vars',[q, param]);
matlabFunction(f_cg,'file','AutoGeneratedFcts\F_CoriGrav','vars',[q, dqdt, param]);

matlabFunction(BodyL,'file','AutoGeneratedFcts\BodyL','vars',[q, param]);
matlabFunction(BodyR,'file','AutoGeneratedFcts\BodyR','vars',[q, param]);
matlabFunction(JBodyL,'file','AutoGeneratedFcts\ContactJacobianL','vars',[q, param]);
matlabFunction(JBodyR,'file','AutoGeneratedFcts\ContactJacobianR','vars',[q, param]);
matlabFunction(CoGs,'file','AutoGeneratedFcts\CoGPositions','vars',[q, param]);
matlabFunction(links,'file','AutoGeneratedFcts\LinkPositions','vars',[q, param]);