function [position,isterminal,direction] = EventsFcn_flight(t,x,lLeg_flag,ter_i)

if lLeg_flag
    position = [(x(2)-x(6)*cos(x(3)+x(7))) - Terrain(x(1)+x(6)*sin(x(3)+x(7)),ter_i),...
                 x(2) - Terrain(x(1),ter_i),...
                (x(2)-x(6)*cos(x(3)+x(7))) - Terrain(x(1)+x(6)*sin(x(3)+x(7)),ter_i)];
else
    position = [(x(2)-x(4)*cos(x(3)+x(5))) - Terrain(x(1)+x(4)*sin(x(3)+x(5)),ter_i),...
                 x(2) - Terrain(x(1),ter_i),...
                (x(2)-x(4)*cos(x(3)+x(5))) - Terrain(x(1)+x(4)*sin(x(3)+x(5)),ter_i)];
end
                % The value that we want to be zero
isterminal = [1, 1, 1];    % Halt integration 
direction = [-1, -1, 1];   % The direction that the zero is approached 