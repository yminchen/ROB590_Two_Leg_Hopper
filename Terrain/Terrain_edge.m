function edge = Terrain_edge(terrain_label)

if terrain_label == 0
    edge = [0 0
            2 0];
        
elseif terrain_label == 1
    edge = [2 0
            4 0.5
            6 0];
    
elseif terrain_label == 2
    edge = [1   2
            1.1 0
            2   0
            4   1
            5   2
            6   2
            7   1.5
            8   2.5
            9   2
            11  0];
    
elseif terrain_label == 3
    edge = [1   2
            1.1 0
            2   0
            4   0.5
            5   1
            6   1
            7   0.75
            8   1.25
            9   1
            11  0];
    
end


% % just for testing, should be deleted later
% if terrain_label == 0
%     edge = [-2 0
%             2 4];
% end