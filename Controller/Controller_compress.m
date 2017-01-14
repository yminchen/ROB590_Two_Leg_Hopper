function F = Controller_compress(x, k1, k2, L, lLeg_flag, k_f, max_dx_des, target_pos, contact_pos, t_prev_stance)
% Function for state feedback controller.
%
% F:   Joint Torque (N*m)
F = zeros(3,1);

%% Body balance
tar_angle = 0; 

% PD controller parameters
kp = 10;    % 1.5
kd = 0.5;   % 1.2
max_f = 200;    % maximum torque that can be applied
% PD controller for desired phi.
if lLeg_flag
    err = x(5)-x(3) - tar_angle;
    derr =  x(10)-x(8);     
else
    err = x(2)-x(3) - tar_angle;
    derr =  x(7)-x(8);     
end
f_final = kp*err + kd*derr;
if f_final > max_f
    f_final = max_f;
elseif f_final < -max_f
    f_final = -max_f;
end

% Assignment
F(1) = f_final;

%% Swing foot angle
% The target angle is the same as the one in flight phase.
% Position controller parameters
kp_pos = k_f(1);
kd_pos = k_f(2);
% Position controller (PD control)
xPos = contact_pos(1) - x(1)*sin(x(2));
xVel = -x(6)*sin(x(2))-x(1)*x(7)*cos(x(2));
dx_des = -kp_pos*(xPos-target_pos) - kd_pos*xVel; % target_pos is fixed.
if dx_des>max_dx_des
    dx_des = max_dx_des;
elseif dx_des<-max_dx_des
    dx_des = -max_dx_des;
end

% dx_des = 2; % for tuning kp_rai

% Raibert style controller parameters
kp_rai = k_f(3);
max_phi_tar = 50*pi/180;
% Raibert style controller
x_des = xVel*t_prev_stance/2 + kp_rai*(xVel-dx_des);
phi_tar = asin(x_des/L);
if phi_tar > max_phi_tar
    phi_tar = max_phi_tar;
elseif phi_tar < -max_phi_tar
    phi_tar = -max_phi_tar;
end

% PD controller parameters
kp = 10;    % 1.5
kd = 0.5;   % 1.2
max_f = 200;    % maximum torque that can be applied
% PD controller for desired phi.
if lLeg_flag
    err = x(5)-x(3)+x(2) - phi_tar;
    derr =  x(10)-x(8)+x(7);     
else
    err = x(2)-x(3)+x(5) - phi_tar;
    derr =  x(7)-x(8)+x(10);     
end
f_final = -kp*err - kd*derr;
if f_final > max_f
    f_final = max_f;
elseif f_final < -max_f
    f_final = -max_f;
end

% Assignment
F(2) = f_final;


%% Swing foot Length

if lLeg_flag
    F(3) = -k1*(L*0.3); % length (retraction)
else
    F(3) = -k2*(L*0.3); % length (retraction)
end
% TODO: The retraction length is dependent on 
%       1. the apex height of body before touchdown and 
%       2. the forward velocity. 
% F(3) can be modified.

