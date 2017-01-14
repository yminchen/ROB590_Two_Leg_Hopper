function dx = F_spring_R(t,x,d,k1,k2,L_sp0,L_mB,mB,IB,m1,m2,lLeg_flag,thrust_flag,E_des,k_des,k_f,max_dx_des,target_pos,contact_pos,t_prev_stance)
% Derivative function for the model.
n = max(size(x,1), size(x,2));
dx = zeros(n,1);

% system parameters:
g = 9.81;   % gravitational constant (m/s^2)

% current energy
% Energy_stance(s,phi,theta,ds,dphi,dtheta,g,k,L_sp0,L_m1,m1,I1)
%E = Energy_stance(x(1),x(2),x(3),x(4),x(5),x(6),g,k,L_sp0,L_m1,m1,I1);

%Controller for thrust  
if thrust_flag %&& (E_des > E) 
    k1 = k_des;
end

% Controller for spring angle 
if thrust_flag 
    F_ctrl = Controller_thrust(x, k1, k2, L_sp0, lLeg_flag);
%     F_ctrl = 0;
%     F_ctrl = Controller_compress(x, k2, L_sp0);
else
    F_ctrl = Controller_compress(x, k1, k2, L_sp0, lLeg_flag, k_f, max_dx_des, target_pos, contact_pos, t_prev_stance);
end
% F_ctrl = [0;0;0]; % this line is for testing the stystem. should be deleted later

% equation of motion
% F_CoriGrav_st_R(s1,phi1,theta,s2,phi2,ds1,dphi1,dtheta,ds2,dphi2,x1,y1,g,k1,k2,L_sp0,L_mB,mB,IB,m2)
% InvMassMatrix_st_R(s1,phi1,theta,s2,phi2,x1,y1,g,k1,k2,L_sp0,L_mB,mB,IB,m2)
f_cg = F_CoriGrav_st_R(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10),g,k1,k2,L_sp0,L_mB,mB,IB,m2);
invM = InvMassMatrix_st_R(x(1),x(2),x(3),x(4),x(5),g,k1,k2,L_sp0,L_mB,mB,IB,m2);
u = [-d*x(6); 0; F_ctrl(1); -d*x(9)+F_ctrl(3); F_ctrl(2)];

dx(1:n/2) = x(n/2+1:n);
dx(n/2+1:n) = invM*(f_cg + u);

