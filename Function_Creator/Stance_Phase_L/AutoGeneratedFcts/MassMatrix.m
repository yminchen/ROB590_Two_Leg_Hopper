function M = MassMatrix(s1,phi1,theta,s2,phi2,x2,y2,g,k1,k2,L_sp0,L_mB,mB,IB,m1)
%MASSMATRIX
%    M = MASSMATRIX(S1,PHI1,THETA,S2,PHI2,X2,Y2,G,K1,K2,L_SP0,L_MB,MB,IB,M1)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    14-Jan-2017 12:59:23

t2 = phi1-theta;
t3 = s1.^2;
t4 = m1.*t3;
t5 = sin(t2);
t6 = cos(t2);
t7 = m1.*s1.*t5;
t8 = s1-s2.*t6;
t9 = m1.*s1.*t8;
t10 = m1.*s1.*s2.*t6;
t11 = -IB-t4+t10;
t12 = s2.^2;
M = reshape([m1,0.0,0.0,-m1.*t6,-m1.*s2.*t5,0.0,t4,-t4,t7,t9,0.0,-t4,IB+t4,-t7,t11,-m1.*t6,t7,-t7,m1+mB,t7,-m1.*s2.*t5,t9,t11,t7,IB+t4+m1.*t12+mB.*t12-m1.*s1.*s2.*t6.*2.0],[5,5]);
