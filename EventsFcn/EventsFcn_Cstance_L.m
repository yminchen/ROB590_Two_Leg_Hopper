function [position,isterminal,direction] = EventsFcn_Cstance_L(t,x,contact_pos,ter_i)

position = [x(9),...
    contact_pos(2)+x(4)*cos(x(5)) - Terrain(contact_pos(1)-x(4)*sin(x(5)),ter_i)]; 
    % The value that we want to be zero
isterminal = [1, 1];  % Halt integration 
direction = [1, -1];   % The direction that the zero is approached 