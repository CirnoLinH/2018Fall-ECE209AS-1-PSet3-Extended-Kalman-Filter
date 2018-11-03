function y = boundary_eval(state,L,W,boundary)

% Boundary in the form of [X, X]:
% X indicates the line that a line from robot in the heading direction
% would intersect with.

x = state(1);
y = state(2);
angle = state(3);

x_temp = x + (boundary(2) - y)/tan(angle);
y_temp = y + (boundary(1) - x)*tan(angle);


if y_temp >= 0 && y_temp <= W
    
    y = [boundary(1),y_temp];
    
else
    
    y = [x_temp,boundary(2)];
end

end