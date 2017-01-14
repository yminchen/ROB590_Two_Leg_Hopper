function F = Controller_thrust(x, k1, k2, L, lLeg_flag)
% Function for state feedback controller.
%
% F:   Joint Torque (N*m)
F = zeros(3,1);

%% Body balance
tar_vel = 0; 

% P controller parameters
kp = 2;    % 1.5
% kd = 0.2;   % 1.2
max_f = 200;    % maximum torque that can be applied
% P controller for desired angular velocity.
if lLeg_flag
    err = x(10)-x(8) - tar_vel;
else
    err = x(7)-x(8) - tar_vel;
end
% derr =  x(5)-x(6);
f_final = kp*err;% + kd*derr;
if f_final > max_f
    f_final = max_f;
elseif f_final < -max_f
    f_final = -max_f;
end

% Assignment
F(1) = f_final;


%% Swing foot angle
tar_vel = 0; 

% P controller parameters
kp = 10;    % 1.5
%kd = 0.5;   % 1.2
max_f = 200;    % maximum torque that can be applied
% PD controller for desired phi.
if lLeg_flag
    err = x(10)-x(8)+x(7) - tar_vel;
    %derr =  x(7)-x(8);     
else
    err = x(7)-x(8)+x(10) - tar_vel;
    %derr =  x(10)-x(8);  
end
f_final = -kp*err;% + kd*derr;
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
