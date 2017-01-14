function invM = InvMassMatrix(s1,phi1,theta,s2,phi2,x1,y1,g,k1,k2,L_sp0,L_mB,mB,IB,m2)
%INVMASSMATRIX
%    INVM = INVMASSMATRIX(S1,PHI1,THETA,S2,PHI2,X1,Y1,G,K1,K2,L_SP0,L_MB,MB,IB,M2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    14-Jan-2017 01:29:05

t2 = 1.0./mB;
t3 = phi2-theta;
t4 = 1.0./s1.^2;
t5 = t2.*t4;
t6 = sin(t3);
t7 = 1.0./s1;
t8 = 1.0./s2;
t9 = cos(t3);
t10 = t2.*t6.*t7;
t11 = 1.0./IB;
t12 = t2.*t9;
t13 = t2.*t7.*t8.*t9;
t14 = IB.*t9;
t15 = mB.*s1.*s2;
t16 = t14+t15;
t17 = t2.*t7.*t8.*t11.*t16;
t18 = 1.0./m2;
invM = reshape([t2,0.0,0.0,t12,-t2.*t6.*t8,0.0,t5,t5,t10,t13,0.0,t5,t2.*t4.*t11.*(IB+mB.*s1.^2),t10,t17,t12,t10,t10,t2.*t18.*(m2+mB),0.0,-t2.*t6.*t8,t13,t17,0.0,1.0./s2.^2.*t2.*t11.*t18.*(IB.*m2+IB.*mB+m2.*mB.*s2.^2)],[5,5]);
