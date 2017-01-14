function y = Terrain(x,terrain_label)
% terrain funcrion y(x)
% y(x0) is the height of the ground at x = x0.

% The size of y is the same as that of x.
% Both of them would be row vectors or column vectors.

n = size(x);
y = zeros(n);

n1 = size(y,1);
n2 = size(y,2);
if n1<n2
    n1 = n2;
end

% if terrain_label = 0, just output a flat terrain.
if terrain_label 
    edge = Terrain_edge(terrain_label);
    for i=1:n1    
        for j = 1:size(edge,1)+1
            if j==size(edge,1)+1
                y(i) = edge(j-1,2);
            elseif x(i)<edge(j,1) 
                if j==1 
                    y(i) = edge(j,2);
                else
                    y(i) = (edge(j,2)-edge(j-1,2))/(edge(j,1)-edge(j-1,1))*...
                        (x(i)-edge(j-1,1)) + edge(j-1,2);
                end
                break;
            end
        end
    end
end

% % just for testing, should be deleted later
% if ~terrain_label
%     y = x+2;
% end
