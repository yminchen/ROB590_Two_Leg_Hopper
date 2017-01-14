function CoGs = CoGPositions(xB,yB,theta,s1,phi1,s2,phi2,g,k1,k2,L_sp0,L_mB,mB,IB,m1,m2)
%COGPOSITIONS
%    COGS = COGPOSITIONS(XB,YB,THETA,S1,PHI1,S2,PHI2,G,K1,K2,L_SP0,L_MB,MB,IB,M1,M2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    14-Jan-2017 01:12:21

t2 = phi1+theta;
t3 = phi2+theta;
CoGs = reshape([xB,yB,theta,xB+s1.*sin(t2),yB-s1.*cos(t2),0.0,xB+s2.*sin(t3),yB-s2.*cos(t3),0.0],[3,3]);
