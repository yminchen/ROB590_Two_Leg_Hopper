function JBodyL = ContactJacobianL(xB,yB,theta,s1,phi1,s2,phi2,g,k1,k2,L_sp0,L_mB,mB,IB,m1,m2)
%CONTACTJACOBIANL
%    JBODYL = CONTACTJACOBIANL(XB,YB,THETA,S1,PHI1,S2,PHI2,G,K1,K2,L_SP0,L_MB,MB,IB,M1,M2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    14-Jan-2017 01:12:21

JBodyL = reshape([1.0,0.0,0.0,1.0,L_mB.*sin(theta).*(1.0./2.0),L_mB.*cos(theta).*(-1.0./2.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[2,7]);
