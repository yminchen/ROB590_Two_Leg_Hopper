function [position,isterminal,direction] = EventsFcn_Tstance_L(t,x,contact_pos,ter_i,k2,L_sp0,m2)

g = 9.81;

position = [k2*(x(4)-L_sp0)*cos(x(5))-m2*g,...
    contact_pos(2)+x(4)*cos(x(5)) - Terrain(contact_pos(1)-x(4)*sin(x(5)),ter_i)]; 
    % The value that we want to be zero
isterminal = [1, 1];  % Halt integration 
direction = [1, -1];   % The direction that the zero is approached 