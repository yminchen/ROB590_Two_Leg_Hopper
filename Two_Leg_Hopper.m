%% Two-leg Hopper model jumping in two dimension.

% Author: Yu-Ming Chen, University of Michigan, Ann Arbor
% Email: yminchen@umich.edu
% Date: 01/14/2017

% This code is modified from an example 
% "Simulation and Animation of a Linear and Nonlinear Pendulum Model"
% written by 
% James T. Allison, Assistant Professor, University of Illinois at
% Urbana-Champaign.


%% INITIALIZATION:
clear,clc
% add path (so we can have access to functions in the folders)
addpath('EOM','EventsFcn','Controller','Animation','Terrain');

% settings
fflag = 1;          % flag to enable figure plot (don't enable this with animation at the same time)
aflag = not(fflag); % flag to enable anima3tion creation
vflag = not(fflag); % flag to enable animation recording (create a avi video)
fignum = 1;         % figure number
moviename = 'Two_Leg_Hopper.avi';  % avi file name for VideoWriter
subplotFlag = 0;    % flag to enable animation subplot

% system parameters
mB = 1;             % body mass (kg)
L_mB = 0.5;         % body length (m)
IB = mB*L_mB^2/12;  % body inertia (kg*m^2)
m1 = 0.01;          % right foot mass (kg)
m2 = 0.01;          % left foot mass (kg)
L_sp0 = 0.7;        % spring original length (m)
k1 = 500;           % right spring constant   
k2 = 500;           % left spring constant   
        % higher stiffness improves stability.
        % if you want higher speed, you need higher k.
d = 10;             % spring damping 
g = 9.81;           % gravitational constant (m/s^2)
ter_i = 0;          % terrain label
k = 0;
% controller parameters
target_pos = 10;
t_prev_stance = 0.2/(k1/100);
H = 1.2;            % desired height (effecting kp_rai and kp_pos!)
max_dx_des = 1;     % maximum of desired speed (not real speed)
        % max_dx_des can go to 8 or higher, but then it also jumps higher.
dx_des = 0;         % desired speed (initialized to be 0)
E_low = 0;          % energy at lowest point (initialized to be 0)
E_des = 0;          % desired energy (initialized to be 0)
L_sp_low = 0;       % spring length when mass reaches lowest height 
k_des = 0;          % desired spring constant during the thrust phase
x_td = 0;           % state vector at previous touchdown 
prev_t = 0;
% position controller parameters
kp_pos = 2;       
        % kp_pos depends on max_dx_des.
kd_pos = 1.5;
% Raibert controller parameter
kp_rai = 0.04;      % Raibert sytle controller
        % kp_rai depends on H (the height) and k (the stiffness).
        % For larger desired speed, you need higher kp_rai.
        % But the larger kp_rai is, the more unstable when changing speed.
        % When it's stable enough, you can try to increase max_dx_des.
k_f = [kp_pos kd_pos kp_rai];  % f stands for flight

% simulation parameters
T0 = 0;
Tf = 15;
% ode45 events parameters
t_evMax = 1;
% initial simulation parameters
tstep = 0.01;               % time increment (s)
tspan = T0:tstep:t_evMax;   % initial time vector
        % Including intermidiate time allows us to fix the 
        % length of ode output. 

% state vector
% Note that x0 is a row vector.
% Flight phase has 14 states (7 DOF), and stance phase 10 (5 DOF):
% 1. flight phase
x0 = [  0       % xB    (m)     ; horizontal position of body (mB) 
        1.2+Terrain(0,ter_i)
                % yB    (m)     ; vertical position of body (mB)
        %L_sp0-0.0049+0.0001
        0       % theta (rad)   ; angle of mB w.r.t horizontal line
        L_sp0   % s1    (m)     ; length of spring
        %L_sp0-0.0049
        0       % phi1  (rad)   ; angle of spring w.r.t the norm of body (mB)
        L_sp0   % s2    (m)     ; length of spring
        %L_sp0-0.0049
        0       % phi2  (rad)   ; angle of spring w.r.t the norm of body (mB)
        0       % d_xB  (m/s)
        0       % d_yB  (m/s)
        0       % d_theta (rad/s)
        0       % d_s1  (m/s)
        0       % d_phi1(rad/s)
        0       % d_s2  (m/s)
        0]';    % d_phi2(rad/s)
% 2. stance phase
                % s1    (m)     ; length of spring
                % phi1  (rad)   ; angle of the spring w.r.t. vertical line
                % theta (rad)   ; angle of body w.r.t. the norm of spring
                % s2    (m)     ; length of spring
                % phi2  (rad)   ; angle of spring w.r.t the norm of body (mB)
                % d_s1  (m/s)
                % d_phi1(rad/s)
                % d_theta (rad/s)
                % d_s2  (m/s)
                % d_phi2(rad/s)
                

% contact position of spring and ground
contact_pos = zeros(2,1);

% finite state machine
lLeg_flag = 0;      % 0 is right leg, and 1 is left leg.
phase = 0;          % 0 is flight phase, and 1 is stance phase
thrust_flag = 0;    % 0 is in thrust phase, 1 is not.
                    
% output value        
T = zeros(0,0);     % t_output (will have one column)
T(1,1) = T0;
X = zeros(0,0);     % x_output (will have six columns)
X(1,:) = x0;
S = zeros(0,0);     % keep track of states status
S(1,1) = 1;         % 1 is flight, 2 is comp, 3 is thrust; right leg
                    % 4 is flight, 5 is comp, 6 is thrust; left leg
lenX = 1;           % the number of rows of x_output 
                    
% plotting parameters
line_height = 7;

% create figure window for plotting
if fflag
    figure;
end 


%% SIMULATION:

while T(size(T,1)) < Tf
    %%% flight phase %%%
    if phase == 0 
        
        options = odeset('Events', @(t,x) EventsFcn_flight(t,x,lLeg_flag,ter_i));
        [t,x,te,xe,ie] = ode45(@(t,x) F_freefall(t,x,d,k1,k2,L_sp0,L_mB,mB,IB,m1,m2,...
            lLeg_flag,t_prev_stance,target_pos,k_f,max_dx_des), tspan, x0, options);
        
        n = size(x,1);
        % assign output
        X(lenX:lenX+n-1, :) = x;
        T(lenX:lenX+n-1, :) = t;
        if lLeg_flag
            S(lenX:lenX+n-1, :) = 4*ones(n,1);
        else
            S(lenX:lenX+n-1, :) = ones(n,1);
        end
        
        % update the time info and initial state for ode45
        lenX = lenX + n-1;
        tspan = t(n) : tstep : t(n)+t_evMax;
        x0 = x(n, :);
        
        % If the point mass falls on the ground, stop the simulation.
        if (size(te,1)>0) && (ie(size(ie,1))==2)
            break;
        % If the foot is hitting the ground, switch to stance phase.
        elseif (size(te,1)>0) && (ie(size(ie,1))==1)
            phase = 1;
            % print
            if lLeg_flag
                display('switch to compression (left)');
            else
                display('switch to compression (right)');
            end
            
            % contact point
            if lLeg_flag
                contact_pos = [ x(n,1) + x(n,6)*sin(x(n,3)+x(n,7)); 
                                x(n,2) - x(n,6)*cos(x(n,3)+x(n,7))];
            else
                contact_pos = [ x(n,1) + x(n,4)*sin(x(n,3)+x(n,5)); 
                                x(n,2) - x(n,4)*cos(x(n,3)+x(n,5))];
            end
            % transition info
            prev_t = t(n);
            % flight controller info
            dx_des = -k_f(1)*((x(n,1)+x(n,8)*t_prev_stance/2)-target_pos)...
                     -k_f(2)*x(n,8);
            if dx_des>max_dx_des
                dx_des = max_dx_des;
            elseif dx_des<-max_dx_des
                dx_des = -max_dx_des;
            end
            % state vector at touch down
            x_td = x(n,:);
            
        elseif (size(te,1)>0)
%             display('coming out from the ground. incorrect dynamics.');
%             break;
        end
        
    %%% stance phase %%%
    elseif phase == 1
        
        % if left leg is the stance leg.
        if lLeg_flag
            if thrust_flag
                options = odeset('Events', @(t,x) EventsFcn_Tstance_L(t,x,contact_pos,ter_i,k2,L_sp0,m2));
            else
                options = odeset('Events', @(t,x) EventsFcn_Cstance_L(t,x,contact_pos,ter_i));
            end

            % transform to radius-tangent coordinate
            x0 = [  X(lenX,4)    
                    X(lenX,5)
                    X(lenX,7)
                    X(lenX,6)
                    X(lenX,3)+X(lenX,7) 
                    X(lenX,11)
                    X(lenX,12)
                    (-X(lenX,8)*cos(X(lenX,3)+X(lenX,7))-X(lenX,9)*sin(X(lenX,3)+X(lenX,7)))/X(lenX,6) - X(lenX,10)
                    (-X(lenX,8)*sin(X(lenX,3)+X(lenX,7))+X(lenX,9)*cos(X(lenX,3)+X(lenX,7)))
                    (-X(lenX,8)*cos(X(lenX,3)+X(lenX,7))-X(lenX,9)*sin(X(lenX,3)+X(lenX,7)))/X(lenX,6)]';
            %%% TODO: checkout if the convertion is correctly implemented.
                        
            % ode45
            [t,x,te,xe,ie] = ode45(@(t,x) F_spring_L(t,x,d,k1,k2,L_sp0,L_mB,mB,IB,m1,m2,...
            lLeg_flag,thrust_flag,E_des,k_des,k_f,max_dx_des,target_pos,contact_pos,t_prev_stance), tspan, x0, options);

            n = size(x,1);

            % transform to x-y coordinate
            x = [   contact_pos(1) - x(:,4).*sin(x(:,5)),...
                    contact_pos(2) + x(:,4).*cos(x(:,5)),...
                    x(:,5)-x(:,3),...
                    x(:,1),...
                    x(:,2),...
                    x(:,4),...
                    x(:,3),...
                    -x(:,9).*sin(x(:,5))-x(:,4).*x(:,10).*cos(x(:,5)),...
                    +x(:,9).*cos(x(:,5))-x(:,4).*x(:,10).*sin(x(:,5)),...
                    x(:,10)-x(:,8),...
                    x(:,6),...
                    x(:,7),...
                    x(:,9),...                    
                    x(:,8)];
        
        % if right leg is the stance leg.
        else
            if thrust_flag
                options = odeset('Events', @(t,x) EventsFcn_Tstance_R(t,x,contact_pos,ter_i,k1,L_sp0,m1));
            else
                options = odeset('Events', @(t,x) EventsFcn_Cstance_R(t,x,contact_pos,ter_i));
            end

            % transform to radius-tangent coordinate
            x0 = [  X(lenX,4)    
                    X(lenX,3)+X(lenX,5)
                    X(lenX,5)
                    X(lenX,6)   
                    X(lenX,7)    
                    -X(lenX,8)*sin(X(lenX,3)+X(lenX,5))+X(lenX,9)*cos(X(lenX,3)+X(lenX,5))  
                    (-X(lenX,8)*cos(X(lenX,3)+X(lenX,5))-X(lenX,9)*sin(X(lenX,3)+X(lenX,5)))/X(lenX,4)
                    (-X(lenX,8)*cos(X(lenX,3)+X(lenX,5))-X(lenX,9)*sin(X(lenX,3)+X(lenX,5)))/X(lenX,4) - X(lenX,10)
                    X(lenX,13)
                    X(lenX,14)]';
            %%% TODO: checkout if the convertion is correctly implemented.

            % ode45
            [t,x,te,xe,ie] = ode45(@(t,x) F_spring_R(t,x,d,k1,k2,L_sp0,L_mB,mB,IB,m1,m2,...
            lLeg_flag,thrust_flag,E_des,k_des,k_f,max_dx_des,target_pos,contact_pos,t_prev_stance), tspan, x0, options);

            n = size(x,1);

            % transform to x-y coordinate
            x = [   contact_pos(1) - x(:,1).*sin(x(:,2)),...
                    contact_pos(2) + x(:,1).*cos(x(:,2)),...
                    x(:,2)-x(:,3),...
                    x(:,1),...
                    x(:,3),...
                    x(:,4),...
                    x(:,5),...
                    -x(:,6).*sin(x(:,2))-x(:,1).*x(:,7).*cos(x(:,2)),...
                    +x(:,6).*cos(x(:,2))-x(:,1).*x(:,7).*sin(x(:,2)),...
                    x(:,7)-x(:,8),...
                    x(:,6),...
                    x(:,8),...
                    x(:,9),...                    
                    x(:,10)];
        end
        
        % assign output
        X(lenX:lenX+n-1, :) = x;
        T(lenX:lenX+n-1, :) = t;
        if thrust_flag
            if lLeg_flag
                S(lenX:lenX+n-1, :) = 6*ones(n,1);
            else
                S(lenX:lenX+n-1, :) = 3*ones(n,1);
            end
        else
            if lLeg_flag
                S(lenX:lenX+n-1, :) = 5*ones(n,1);
            else
                S(lenX:lenX+n-1, :) = 2*ones(n,1);
            end
        end
        
        % update the time info and initial state for ode45
        lenX = lenX + n-1;
        tspan = t(n) : tstep : t(n)+t_evMax;
        x0 = x(n, :);        
        
        % If the point mass falls on the ground, stop the simulation.
        if (size(te,1)>0) && (ie==2)
            break;
            
        % If the foot is leaving the ground, switch to flight phase.
        elseif (size(te,1)>0) && (thrust_flag==1)
            phase = 0;
            thrust_flag = 0;
            lLeg_flag = ~lLeg_flag;
            % print
            if lLeg_flag
                display('switch to flight (left)');
            else
                display('switch to flight (right)');
            end
            
            % transition info and plot
            t_prev_stance = t(n) - prev_t;
            if fflag
                hold on;
                fp(1) = fill([prev_t prev_t t(n) t(n)],[-line_height,line_height,line_height,-line_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
                hold off;
            end
            
        % If the body reaches the lowest hieght, switch to thrust phase.
        elseif (size(te,1)>0) && (thrust_flag == 0)
            thrust_flag = 1;
            % print
            if lLeg_flag
                display('switch to thrust (left)');
            else
                display('switch to thrust (right)');
            end
            
            % calculate spring length and energy at lowest height
            L_sp_low = sum(([x(n,1);x(n,2)]-contact_pos).^2)^0.5;   
            if lLeg_flag
                E_low = mB*g*x(n,2) + 0.5*mB*(x(n,8)^2+x(n,9)^2) +...
                        0.5*IB*(x(n,10))^2 + 0.5*k2*(L_sp_low - L_sp0)^2; % not including swing leg's
            else
                E_low = mB*g*x(n,2) + 0.5*mB*(x(n,8)^2+x(n,9)^2) +...
                        0.5*IB*(x(n,10))^2 + 0.5*k1*(L_sp_low - L_sp0)^2; % not including swing leg's
            end
            % calculate desired energy (not including swing leg's)
            if lLeg_flag
                dL = abs(-x_td(8)*sin(x_td(3)+x_td(7))+x_td(9)*cos(x_td(3)+x_td(7))); %length speed at touch down
            else
                dL = abs(-x_td(8)*sin(x_td(3)+x_td(5))+x_td(9)*cos(x_td(3)+x_td(5))); %length speed at touch down
            end
            E_des = mB*g*(H+Terrain(x(n,1)+x(n,8)*t_prev_stance/2,ter_i))...
                + pi/4*d*dL*(L_sp0-L_sp_low) + 0.5*mB*dx_des^2;
            
            if lLeg_flag
                k_des = k2 + 2*(E_des-E_low)/(L_sp0-L_sp_low)^2;
                if k_des < k2
                    k_des = k2;
                end
            else
                k_des = k1 + 2*(E_des-E_low)/(L_sp0-L_sp_low)^2;
                if k_des < k1
                    k_des = k1;
                end
            end
        end
    end

end

%%%% TODO: In order to make the time in the video super accurate, 
%%%%       we should record the output every 0.01 second. 
%%%%       So don't assign the event time to the output (both X and T)
%%%%       (although we can assign it to another variable).
%%%% FYI: frame rate is decided by "vidObj.FrameRate"

%% Trajectary Plot
if fflag
    if phase == 1
        hold on;
        fp(1) = fill([prev_t prev_t tspan(1) tspan(1)],[-line_height,line_height,line_height,-line_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
        hold off;
    end
        
    hold on;
    fp(2) = plot(T,X(:,1),'b','LineWidth',2);
    fp(3) = plot(T,X(:,2),'r','LineWidth',2);
    fp(4) = plot(T,X(:,3),'g','LineWidth',2);
    fp(5) = plot(T,X(:,4),'k','LineWidth',2);
    fp(6) = plot(T,X(:,5),'m','LineWidth',2);
    fp(7) = plot(T,X(:,4),'y','LineWidth',2);
    fp(8) = plot(T,X(:,5),'c','LineWidth',2);
    fp(9) = plot(T,X(:,8),'b--','LineWidth',1);
    fp(10) = plot(T,X(:,9),'r--','LineWidth',1);
    fp(11) = plot(T,X(:,10),'g--','LineWidth',1);
    fp(12) = plot(T,X(:,11),'k--','LineWidth',1);
    fp(13) = plot(T,X(:,12),'m--','LineWidth',1);
    fp(14) = plot(T,X(:,13),'y--','LineWidth',1);
    fp(15) = plot(T,X(:,14),'c--','LineWidth',1);
    fp(16) = plot([T(1) T(size(T,1))], [target_pos target_pos],'r--','LineWidth',1);
    hold off
    
    if d == 0
        title('\fontsize{12}\fontname{Arial Black}Two-leg Hopper without Damping (2D)')
    else
        title('\fontsize{12}\fontname{Arial Black}Two-leg Hopper with Damping (2D)')        
    end
    legend(fp([1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16]), 'stance phase',...
        'xPos (m)'  ,'yPos (m)'  ,'theta (rad)'   ,'s1 (m)'   ,'phi1 (rad)'   ,'s2 (m)'   ,'phi2 (rad)',...
        'xVel (m/s)','yVel (m/s)','dtheta (rad/s)','ds1 (m/s)','dphi1 (rad/s)','ds2 (m/s)','dphi2 (rad/s)', 'xPos_{des} (m)');
    xlabel('\fontsize{10}\fontname{Arial Black} Time(s)');
    text(1,9,['\fontsize{10}\fontname{Arial Black}kp_{pos}: ' num2str(k_f(1),'%1.2f')], 'Color','red');
    text(1,8.5,['\fontsize{10}\fontname{Arial Black}kd_{pos} : ' num2str(k_f(2),'%1.2f')], 'Color','red');
    text(1,8,['\fontsize{10}\fontname{Arial Black}kp_{rai} : ' num2str(k_f(3),'%1.2f')], 'Color','red');
%     axis([0 T(size(T,1)) 0 2]);
    
    figure;
    plot(T,X(:,3),'g','LineWidth',2); hold on;
    plot(T,X(:,5),'m','LineWidth',2); hold on;
    plot(T,X(:,10),'g--','LineWidth',1); hold on;
    plot(T,X(:,12),'m--','LineWidth',1);
    legend('theta (rad)' ,'phi (rad)', 'dtheta (rad/s)','dphi (rad/s)');
    title('\fontsize{12}\fontname{Arial Black}Angle plot');
    
    figure;
    plot(T,X(:,3)+X(:,5),'m','LineWidth',2);hold on;
    plot(T,X(:,10)+X(:,12),'c','LineWidth',2);
    plot([T(1) T(size(T,1))], [1 1],'r--','LineWidth',1);
    legend('theta+phi (rad)', 'dtheta+dphi (rad/s), ', 'target');
    title('\fontsize{12}\fontname{Arial Black} Global angle plot');
    axis([0 T(size(T,1)) -2 2]);
    
    figure;
    plot(T,X(:,4),'k','LineWidth',2);hold on;
    plot(T,X(:,11),'k--','LineWidth',1);
    legend('s (m)', 'ds (m/s)');
    title('\fontsize{12}\fontname{Arial Black}spring length plot');
    axis([0 T(size(T,1)) -2 2]);
    
    figure;
    plot(X(:,1),X(:,2),'b','LineWidth',2); 
    title('\fontsize{12}\fontname{Arial Black}Trajectory of 2D Two-leg Hopper');
    ylabel('\fontsize{10}\fontname{Arial Black} (m)');
    xlabel('\fontsize{10}\fontname{Arial Black} (m)');
end


%% POSTPROCESSING:

% Calculate max min velocity
Vmax = max(X(:,8));
Vmin = min(X(:,8));

% position plot window size
if aflag
    dt = (Tf-0)/8;
    ndt = floor(dt/tstep);   % number of time increments displayed in window
    h=figure(fignum); clf
    set(h,'Position',[0 0 1000 400]);
end

%% Animation
if aflag
    % step through each time increment and plot results
    if vflag
        vidObj = VideoWriter(moviename);
        vidObj.FrameRate = length(T)/(Tf-0);
        open(vidObj);
        F(length(T)).cdata = []; F(length(T)).colormap = []; % preallocate
    end
    
    boarderR = max(X(:,1))+1;
    boarderL = min(X(:,1))-2;
    boarderT = max(X(:,2))+0.5;
    % This for loop would show animation as well as store the animation in F().
    for ti=1:length(T)
        % State at T(ti)
        xB = X(ti);              

        %%%%% Prepare figure %%%%
        figure(fignum); clf; 

        %%%%% Plot physical world (1st subfigure) %%%%
        if subplotFlag
           subplot(1,3,1);     
           subplot('Position',[0.05 0.05 0.30 0.95]); 
        end
        hold on
        axis equal; axis([boarderL boarderR -0.1 boarderT])

        % Plot ground
        edge = Terrain_edge(ter_i);
        groundL = floor(boarderL);
        groundR = ceil(boarderR);
        for i = 1:size(edge,1)+1
            if i == 1
                fill([groundL edge(i,1) edge(i,1) groundL],[Terrain(groundL,ter_i) Terrain(edge(i,1),ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
            elseif i == size(edge,1)+1
                fill([edge(i-1,1) groundR groundR edge(i-1,1)],[Terrain(edge(i-1,1),ter_i) Terrain(groundR,ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
            else
                fill([edge(i-1,1) edge(i,1) edge(i,1) edge(i-1,1)],[Terrain(edge(i-1,1),ter_i) Terrain(edge(i,1),ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
            end
        end
        % Plot target position
        scatter(target_pos,Terrain(target_pos,ter_i),50,'MarkerEdgeColor','b',...
                  'MarkerFaceColor','g',...
                  'LineWidth',1);
        
        % plot robot
        % links: [BodyL, BodyR, CoG_mB, CoG_m1, CoG_m2]
        links = LinkPositions(X(ti,1),X(ti,2),X(ti,3),X(ti,4),X(ti,5),X(ti,6),X(ti,7),L_mB);
        
        % Plot spring2 (left leg)
        plot([links(1,3) links(1,5)],[links(2,3) links(2,5)],'b','LineWidth',2);
        % Plot m2 (left foot)
        scatter(links(1,5), links(2,5),40,'MarkerEdgeColor',[0 0 0],...
                  'MarkerFaceColor',[0 0 1],...
                  'LineWidth',1.5);
        % Plot mB (body)
%         scatter(links(1,3), links(2,3),40,'MarkerEdgeColor',[0 0 0],...
%                   'MarkerFaceColor',[0 0 0],...
%                   'LineWidth',1.5);
        plot([links(1,1) links(1,2)],[links(2,1) links(2,2)],'k','LineWidth',5);
        % Plot spring1 (right leg)
        plot([links(1,3) links(1,4)],[links(2,3) links(2,4)],'r','LineWidth',2);
        % Plot m1 (right foot)
        scatter(links(1,4), links(2,4),40,'MarkerEdgeColor',[0 0 0],...
                  'MarkerFaceColor',[1 0 0],...
                  'LineWidth',1.5);
        % Display time on plot
        tc = T(ti);     % current time
        text(0.8*boarderL,0.2*boarderT,'\fontsize{10}\fontname{Arial Black}elapsed time:','color','k')
        text(0.8*boarderL,0.1*boarderT,['\fontsize{10}\fontname{Arial Black}' ...
            num2str(tc,'%1.1f') ' sec'],'color','k')
        % Display states on plot
        sc = S(ti);     % current state
        text(0.8*boarderL,0.9*boarderT,'\fontsize{10}\fontname{Arial Black}Flight','color',[0.8,0.8,0.8])
        text(0.8*boarderL,0.8*boarderT,'\fontsize{10}\fontname{Arial Black}Compression','color',[0.8,0.8,0.8])
        text(0.8*boarderL,0.7*boarderT,'\fontsize{10}\fontname{Arial Black}Thrust','color',[0.8,0.8,0.8])
        if sc == 1 
            text(0.8*boarderL,0.9*boarderT,'\fontsize{10}\fontname{Arial Black}Flight','color','r')
        elseif sc == 2 
            text(0.8*boarderL,0.8*boarderT,'\fontsize{10}\fontname{Arial Black}Compression','color','r')
        elseif sc == 3 
            text(0.8*boarderL,0.7*boarderT,'\fontsize{10}\fontname{Arial Black}Thrust','color','r')
        elseif sc == 4
            text(0.8*boarderL,0.9*boarderT,'\fontsize{10}\fontname{Arial Black}Flight','color','b')
        elseif sc == 5
            text(0.8*boarderL,0.8*boarderT,'\fontsize{10}\fontname{Arial Black}Compression','color','b')
        elseif sc == 6
            text(0.8*boarderL,0.7*boarderT,'\fontsize{10}\fontname{Arial Black}Thrust','color','b')
        end
        if sc <= 3
            text(0.7*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}R','color','r')
            text(0.8*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}L','color',[0.8,0.8,0.8])
        else
            text(0.8*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}L','color','b')
            text(0.7*boarderL,0.6*boarderT,'\fontsize{10}\fontname{Arial Black}R','color',[0.8,0.8,0.8])
        end
        
        title('\fontsize{12}\fontname{Arial Black}Two-Leg Hopper Animation (2D)')
        xlabel('\fontsize{10}\fontname{Arial Black} (m)')
        ylabel('\fontsize{10}\fontname{Arial Black} (m)')
        
        %%%%% The other two plots [not updated] %%%%
        if subplotFlag
            %%%%% Plot position trajectory (2nd subfigure)%%%%
            subplot(1,3,2);
            subplot('Position',[0.40 0.34 0.125 0.42]); hold on
            axis([-dt dt -0.1 X(1,1)+0.1])

            % obtain time history for position to current time
            if (ti-ndt) >= 1 
                % full time history indices
                thi = ti-ndt:ti;
                x2h = X(thi);
            else
                % partial time history indices
                thi = 1:ti;
                x2h = X(thi);
            end
            th = T(thi);
            % plot position time history 
            plot(th,x2h,'b-','LineWidth',3); hold on
            axis([(tc-dt) (tc+dt) -0.1 X(1,1)+0.1])
            % plot marker
            plot(tc,xB,'k+','MarkerSize',18,'LineWidth',2)
            plot(tc,xB,'ko','MarkerSize',10,'LineWidth',2)
            % plot vertical position line
            plot([(tc-dt) (tc+dt)],[xB xB],'r-')
            % plot zero position line
            plot([(tc-dt) (tc+dt)],[0 0],'k-')

            title('\fontsize{12}\fontname{Arial Black}SLIP Mass Height')
            xlabel('\fontsize{10}\fontname{Arial Black}Time (sec)')
            ylabel('\fontsize{10}\fontname{Arial Black}Height (m)')
            set(gca,'XTick',0:ceil(Tf)+1)
            clear th thi x2h 

            %%%%% plot velocity time history (3rd subfigure)%%%%
            ax(3) = subplot(1,3,3);
            subplot('Position',[0.60 .34 0.125 0.42]); hold on
            axis([-dt dt Vmin-1 Vmax+1])

            % obtain velocity time history to current time
                % nonlinear 
            if (ti-ndt) >= 1 
                % full time history
                thi = ti-ndt:ti;
                Vth = X(thi,2);
            else
                % partial time history
                thi = 1:ti;
                Vth = X(thi,2);
            end
            th = T(thi);
            % plot time history
            plot(th,Vth,'g-','LineWidth',3); hold on
            axis([(tc-dt) (tc+dt) Vmin-1 Vmax+1])
            % plot marker
            plot(tc,X(ti,2),'k+','MarkerSize',18,'LineWidth',2)
            plot(tc,X(ti,2),'ko','MarkerSize',10,'LineWidth',2)
            % plot vertical position line
            plot([(tc-dt) (tc+dt)],[X(ti,2) X(ti,2)],'r-')
            % plot zero velocity line
            plot([(tc-dt) (tc+dt)],[0 0],'k-')

            title('\fontsize{12}\fontname{Arial Black}SLIP Mass Velocity')
            xlabel('\fontsize{10}\fontname{Arial Black}Time (sec)')
            ylabel('\fontsize{10}\fontname{Arial Black}Velocity (m/s)')
            set(gca,'XTick',0:ceil(Tf)+1)
            set(gcf,'Color','w')
            clear th thi Vth 
        end

        h=figure(fignum);
        F(ti) = getframe(h,[0 0 1000 400]); 
        if vflag
            writeVideo(vidObj,F(ti));
        end

    end

    if vflag
        close(vidObj);
        %moviename2 = 'SLIP_2D(2).avi';      % avi file name for movie2avi
        %movie2avi(F,moviename2,'fps',length(T)/(Tf-t0),'compression','none')
    end

end


