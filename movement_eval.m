function [state_new,F,Q] = movement_eval(state_previous,u,t,sigma_motor,noise_flag)

% radius of the car
r = 20;
% b = 85;
% Add std to w
w1 = u(1,1) + sigma_motor*rand();
w2 = u(1,2) + sigma_motor*rand();

cov_motor = [sigma_motor^2,0;0,sigma_motor^2];

% state: [location at x, location at y, angle]
% F: 3x3 matrix
F = zeros(3,3);
V = zeros(3,2);
% previous location
angle_pre = state_previous(1,3);
x_pre = state_previous(1,1);
y_pre = state_previous(1,2);

% Estimated Simplified Model

delta_s_left = w1 * r * t;
delta_s_right = w2 * r * t;
delta_s = (delta_s_left + delta_s_right)/2;
delta_angle = (delta_s_right - delta_s_left)/85;
delta_x = delta_s * cos(angle_pre + delta_angle/2);
delta_y = delta_s * sin(angle_pre + delta_angle/2);


F(1,3) = -t*sin(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2)*(10*w1 + 10*w2);
F(2,3) = t*cos(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2)*(10*w1 + 10*w2);
F(1,1) = 1;
F(2,2) = 1;
F(3,3) = 1;

V(1,1) = 10*t*cos(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2) + (2*t^2*sin(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2)*(10*w1 + 10*w2))/17;
V(1,2) = 10*t*cos(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2) - (2*t^2*sin(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2)*(10*w1 + 10*w2))/17;
V(2,1) = 10*t*sin(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2) - (2*t^2*cos(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2)*(10*w1 + 10*w2))/17;
V(2,2) = 10*t*sin(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2) + (2*t^2*cos(angle_pre - (t*((4*w1)/17 - (4*w2)/17))/2)*(10*w1 + 10*w2))/17;
V(3,1) =  -(4*t)/17;
V(3,2) = (4*t)/17;

Q = V * cov_motor * V';

% Accurate Model (Unused)
%
% if(w1==w2)
%     
%     % Motion in a line
%     delta_x = cosd(angle_pre) * w1 * r * t;
%     delta_y = sind(angle_pre) * w1 * r * t;
%     x_new = delta_x;
%     y_new = delta_y;
%     angle_new = angle_pre;
% 
%     F(1,3) = -w1*r*t*cosd(angle_pre);
%     F(2,3) = w1*r*t*sind(angle_pre);
%     F(1,1) = 1;
%     F(2,2) = 1;
%     F(3,3) = 1;
%     
% else
%     
%     % Motion in a curve
%     if (w1 < w2)
%         
%         % Turn Left
%         R = 82.5*w1/(w2-w1);
%         l = 42.5 + R;
%         
%         delta_angle = rad2deg(w1*r*t/R);
%         angle_new = angle_pre + delta_angle;
%         
%         if (angle_new >=360)
%             angle_new = angle_new - fix(angle_new/360) * 360;
%         end
%         
%         delta_x = - l*sind(angle_pre) + l*sind(angle_new);
%         delta_y = l*sind(angle_pre) - l*cosd(angle_new);
%         x_new = x_pre + delta_x;
%         y_new = y_pre + delta_y;
% 
%         
%         
%     else
%         
%         % Turn Right
%         R = 82.5*w2/(w1-w2);
%         l = 42.5 + R;
%         delta_angle = - rad2deg(w2*r*t/R);
%         angle_new = angle_pre + delta_angle;
%         
%         if (angle_new < 0)
%             angle_new = angle_new + fix(angle_new/360) * 360;
%         end
%         delta_x = l*sind(angle_pre) + l*sind(angle_new);
%         delta_y = - l*cosd(angle_pre) + l*cosd(angle_new);
%         x_new = x_pre + delta_x;
%         y_new = y_pre + delta_y;
%         
%         
%     end
%     
% end

state_new(1,1) = x_pre + delta_x;
state_new(1,2) = y_pre + delta_y;
state_new(1,3) = angle_pre + delta_angle;

if(noise_flag == 1)
    state_new = state_new + [rand()*0.8-0.4,rand()*0.8-0.4,rand()*0.04-0.02];
end


end