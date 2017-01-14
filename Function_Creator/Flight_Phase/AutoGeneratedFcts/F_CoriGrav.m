function f_cg = F_CoriGrav(xB,yB,theta,s1,phi1,s2,phi2,dxB,dyB,dtheta,ds1,dphi1,ds2,dphi2,g,k1,k2,L_sp0,L_mB,mB,IB,m1,m2)
%F_CORIGRAV
%    F_CG = F_CORIGRAV(XB,YB,THETA,S1,PHI1,S2,PHI2,DXB,DYB,DTHETA,DS1,DPHI1,DS2,DPHI2,G,K1,K2,L_SP0,L_MB,MB,IB,M1,M2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    14-Jan-2017 01:12:20

t2 = phi1+theta;
t3 = cos(t2);
t4 = phi2+theta;
t5 = cos(t4);
t6 = sin(t2);
t7 = dtheta.^2;
t8 = sin(t4);
t9 = dphi1.^2;
t10 = dphi2.^2;
f_cg = [dphi1.*ds1.*m1.*t3.*-2.0-dphi2.*ds2.*m2.*t5.*2.0-ds1.*dtheta.*m1.*t3.*2.0-ds2.*dtheta.*m2.*t5.*2.0+m1.*s1.*t6.*t7+m1.*s1.*t6.*t9+m2.*s2.*t7.*t8+m2.*s2.*t8.*t10+dphi1.*dtheta.*m1.*s1.*t6.*2.0+dphi2.*dtheta.*m2.*s2.*t8.*2.0;-g.*m1-g.*m2-g.*mB-dphi1.*ds1.*m1.*t6.*2.0-dphi2.*ds2.*m2.*t8.*2.0-ds1.*dtheta.*m1.*t6.*2.0-ds2.*dtheta.*m2.*t8.*2.0-m1.*s1.*t3.*t7-m1.*s1.*t3.*t9-m2.*s2.*t5.*t7-m2.*s2.*t5.*t10-dphi1.*dtheta.*m1.*s1.*t3.*2.0-dphi2.*dtheta.*m2.*s2.*t5.*2.0;dphi1.*ds1.*m1.*s1.*-2.0-dphi2.*ds2.*m2.*s2.*2.0-ds1.*dtheta.*m1.*s1.*2.0-ds2.*dtheta.*m2.*s2.*2.0-g.*m1.*s1.*t6-g.*m2.*s2.*t8;L_sp0.*k1-k1.*s1+g.*m1.*t3+m1.*s1.*t7+m1.*s1.*t9+dphi1.*dtheta.*m1.*s1.*2.0;-m1.*s1.*(dphi1.*ds1.*2.0+ds1.*dtheta.*2.0+g.*t6);L_sp0.*k2-k2.*s2+g.*m2.*t5+m2.*s2.*t7+m2.*s2.*t10+dphi2.*dtheta.*m2.*s2.*2.0;-m2.*s2.*(dphi2.*ds2.*2.0+ds2.*dtheta.*2.0+g.*t8)];
