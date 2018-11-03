function [state_observ,H] = observation_eval(state,L,W,R)

% To simplify the calculation, the distance between the right side of the
% car and the wall would be the distance between the center of the car and
% the wall in the "right" direction. The length of axle is not taken into
% consideration.

state = state';
x = state(1);
y = state(2);
angle = mod(state(3),2*pi);

if angle > pi
    angle = angle - 2*pi;
end


% Special cases (horizontal or vertical heading)

if angle==0 || angle == pi/2 || angle == -pi/2 || angle == -pi
    if angle == 0
        state_observ = [ L-x, y, angle];
        H = [-1 0 0;0 1 0; 0 0 1];
    elseif angle == pi/2
        state_observ = [ W-y, L-x, angle];
        H = [0 -1 0;-1 0 0; 0 0 1];
    elseif angle == -pi/2
        state_observ = [ y, x, angle];
        H = [0 1 0;1 0 0;0 0 1];
    elseif angle == -pi
        state_observ = [ x, W-y, angle];
        H = [1 0 0;0 -1 0; 0 0 1];
    end
    
else 
    state_observ = zeros(3,1);
    % d1: the distance from the front to the wall in straight line
    % d1: the distance from the right side to the wall in straight line
    state_2 = [x,y,angle_normalize(angle-pi/2)];
    
    if angle>0 && angle <pi/2
        d1 = boundary_eval(state,L,W,[L,W]);
        d2 = boundary_eval(state_2,L,W,[L,0]);
        
    elseif angle>pi/2 && angle <pi
        d1 = boundary_eval(state,L,W,[0,W]);
        d2 = boundary_eval(state_2,L,W,[L,W]);
        
    elseif angle>-pi/2 && angle <0
        d1 = boundary_eval(state,L,W,[L,0]);
        d2 = boundary_eval(state_2,L,W,[0,0]);
        
    elseif angle>-pi && angle <-pi/2
        d1 = boundary_eval(state,L,W,[0,0]);
        d2 = boundary_eval(state_2,L,W,[0,W]);
      
    end
    
    state_observ(1) = sqrt((d1(1) - x)^2 + (d1(2) - y)^2);
    state_observ(2) = sqrt((d2(1) - x)^2 + (d2(2) - y)^2);
    state_observ(3) = angle;
    
    H = [ -(2*d1(1) - 2*x)/(2*((d1(1) - x)^2 + (d1(2) - y)^2)^(1/2)), -(2*d1(2) - 2*y)/(2*((d1(1) - x)^2 + (d1(2) - y)^2)^(1/2)), 0; ...
          -(2*d2(1) - 2*x)/(2*((d2(1) - x)^2 + (d2(2) - y)^2)^(1/2)), -(2*d2(2) - 2*y)/(2*((d2(1) - x)^2 + (d2(2) - y)^2)^(1/2)), 0; ...
                                                               0,                                                       0, 1];  
end

state_observ = (state_observ + sqrt(R) * normrnd(0,1,3,1))';



end

