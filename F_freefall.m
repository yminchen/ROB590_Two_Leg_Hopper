function dx = F_freefall(t,x,d,k1,k2,L_sp0,L_mB,mB,IB,m1,m2,lLeg_flag,t_prev_stance,target_pos,k_f,max_dx_des)
% Derivative function for the model.
n = max(size(x,1), size(x,2));
dx = zeros(n,1);

% system parameters:
g = 9.81;   % gravitational constant (m/s^2)

% Controller
F_ctrl = Controller_flight(x, k1, k2, L_sp0, lLeg_flag, t_prev_stance, target_pos, k_f, max_dx_des);

% equation of motion
% InvMassMatrix_fl(xB,yB,theta,s1,phi1,s2,phi2,g,k1,k2,L_sp0,L_mB,mB,IB,m1,m2)
% F_CoriGrav_fl(xB,yB,theta,s1,phi1,s2,phi2,dxB,dyB,dtheta,ds1,dphi1,ds2,dphi2,g,k1,k2,L_sp0,L_mB,mB,IB,m1,m2)
invM = InvMassMatrix_fl(x(1),x(2),x(3),x(4),x(5),x(6),x(7),g,k1,k2,L_sp0,L_mB,mB,IB,m1,m2);
f_cg = F_CoriGrav_fl(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10),x(11),x(12),x(13),x(14),g,k1,k2,L_sp0,L_mB,mB,IB,m1,m2);
if lLeg_flag
    u = [0; 0; 0; -d*x(11)+F_ctrl(3); F_ctrl(2); -d*x(13); F_ctrl(1)];
else
    u = [0; 0; 0; -d*x(11); F_ctrl(1); -d*x(13)+F_ctrl(3); F_ctrl(2)];
end

dx(1:n/2) = x(n/2+1:n);
dx(n/2+1:n) = invM*(f_cg + u);