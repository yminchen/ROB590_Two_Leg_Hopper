function [position,isterminal,direction] = EventsFcn_Cstance_R(t,x,contact_pos,ter_i)

position = [x(6),...
    contact_pos(2)+x(1)*cos(x(2)) - Terrain(contact_pos(1)-x(1)*sin(x(2)),ter_i)]; 
    % The value that we want to be zero
isterminal = [1, 1];  % Halt integration 
direction = [1, -1];   % The direction that the zero is approached 