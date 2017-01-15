function F = Controller_flight(x, k1, k2, L, lLeg_flag, postApex_flag, t_prev_stance, target_pos, k_f, max_dx_des)
% Function for state feedback controller.
%
% F(1):   Joint Torque for front foot   (N*m)
% F(2):   Force for retracting front foot (N)
% F(3):   Joint Torque for rear foot    (N*m)
% F(4):   Force for retracting rear foot  (N)
F = zeros(4,1);

%% Front foot (orientation)
% Position controller parameters
kp_pos = k_f(1);
kd_pos = k_f(2);
% Position controller (PD control)
dx_des = -kp_pos*(x(1)-target_pos) - kd_pos*x(8); % target_pos is fixed.
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
x_des = x(8)*t_prev_stance/2 + kp_rai*(x(8)-dx_des);
phi_tar = asin(x_des/L);
if phi_tar > max_phi_tar
    phi_tar = max_phi_tar;
elseif phi_tar < -max_phi_tar
    phi_tar = -max_phi_tar;
end

% phi_tar = 1; % for tuning the next PD controller

% PD controller parameters
kp = 10;   % 1.5
kd = 0.5;   % 1.2
max_f = 200;    % maximum torque that can be applied
% PD controller for desired phi.
if lLeg_flag
    err = x(3)+x(7) - phi_tar;
    derr =  x(10)+x(14);     
else
    err = x(3)+x(5) - phi_tar;
    derr =  x(10)+x(12);     
end
        %%% TODO: phi_target is dynamic, so I should change derr.
f_final = -kp*err - kd*derr;
if f_final > max_f
    f_final = max_f;
elseif f_final < -max_f
    f_final = -max_f;
end

% Assignment
F(1) = f_final;

%% Front foot (length)
% This could be a policy used for rough terrain
% You can detect the incoming terrain before entering the fligh phase.

% if ~postApex_flag
%     if lLeg_flag
%         F(2) = -k2*(L*0.3); % length (retraction)
%     else
%         F(2) = -k1*(L*0.3); % length (retraction)
%     end
% end

%% Back foot (orientation)

phi_tar = 0; % We want the back foot pointing toward the ground. 

% PD controller parameters
kp = 10;   % 1.5
kd = 0.5;   % 1.2
max_f = 200;    % maximum torque that can be applied
% PD controller for desired phi.
if lLeg_flag
    err = x(3)+x(5) - phi_tar;
    derr =  x(10)+x(12);   
else
    err = x(3)+x(7) - phi_tar;
    derr =  x(10)+x(14);     
end
        %%% TODO: phi_target is dynamic, so I should change derr.
f_final = -kp*err - kd*derr;
if f_final > max_f
    f_final = max_f;
elseif f_final < -max_f
    f_final = -max_f;
end

% Assignment
F(3) = f_final; % orientation

%% Back foot (length)

if lLeg_flag
    F(4) = -k1*(L*0.3); % length (retraction)
else
    F(4) = -k2*(L*0.3); % length (retraction)
end
% TODO: The retraction length is dependent on 
%       1. the apex height of body before touchdown and 
%       2. the forward velocity. 
% F(3) can be modified.



