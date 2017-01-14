function links = LinkPositions(s1,phi1,theta,s2,phi2,x1,y1,g,k1,k2,L_sp0,L_mB,mB,IB,m2)
%LINKPOSITIONS
%    LINKS = LINKPOSITIONS(S1,PHI1,THETA,S2,PHI2,X1,Y1,G,K1,K2,L_SP0,L_MB,MB,IB,M2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    14-Jan-2017 01:29:06

t2 = phi1-theta;
t3 = cos(t2);
t4 = sin(phi1);
t5 = cos(phi1);
t6 = s1.*t5;
t7 = sin(t2);
t8 = phi1+phi2-theta;
links = reshape([x1-L_mB.*t3-s1.*t4,t6-L_mB.*t7,x1+L_mB.*t3-s1.*t4,t6+L_mB.*t7,x1-s1.*t4,t6,x1,y1,x1-s1.*t4+s2.*sin(t8),t6-s2.*cos(t8)],[2,5]);