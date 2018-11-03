%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  ECE 209AS-1 Problem Set 3
%  Hanren Lin
%  Chungyu Chen
%  University of California, Los Angeles

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% MAIN FUNCTION

clear
clc

% Location of sensor is the state point's location.
% state: [location at x, location at y, angle between the facing direction and +x]
% x: 0-750 mm
% y: 0-500 mm
% angle: [0, 2*pi) in rad

W = 500;
L = 750;

% standard deviation of motor (5% of max motor speed, 130r/min)
sigma_motor = 2*pi*0.05;

% standard deviation of range sensor
sigma_range = 10;

% standard deviation of bearing sensor
sigma_bearing = 0.1;

% Initial state & covariance
state_initial = [120,120,0.57];
state_previous = state_initial;

state_predict_init = [110,110,0.4]; 
cov_initial = [1 0 0;0 1 0;0 0 1];
cov_t = cov_initial;

% Time interval
t = 0.1;

u = zeros(300,2);

%%%

% 3 Different Action Parts:
% (1) Linear Motion 
% (2) Turn right
% (3) Linear Motion 

%%%

for num = 1:270
    if (num>=1 && num <100)
        u(num,:) = [1.67,1.67];
    elseif (num>=100 && num <200)
        u(num,:) = [1.2,0.3];
    else
        u(num,:) = [0.8,0.8];
    end
end


% Actual

state_actual(1,:) = state_initial;
observ_actual(1,:) = [718.0690,137.4293,1.0738];

% Predict

% state_predict = zeros(270,3);
% state_predict(1,:) = state_predict_init;
% observ_predict = zeros(300,3);
% [observ_predict(1,:),~] = observation_eval(state_predict_init,sigma_range,sigma_bearing);

for i = 2:1:270
    

    R = [sigma_range^2,0,0;0,sigma_range^2,0;0,0,sigma_bearing^2];
    
    [state_actual(i,:),~,~] = movement_eval(state_actual(i-1,:),u(i-1,:),t,sigma_motor,1); % Actual
%     debug_s = F*state_actual(i-1,:)';
%     state_predict_temp1 = (F*state_predict(i-1,:)')';
%     state_predict_temp = state_predict_temp1;

    [state_predict(i-1,:),F,Q] = movement_eval(state_actual(i,:),u(i-1,:),t,sigma_motor,0); % Prediction

    % Observation
    
    [observ_actual(i,:),H] = observation_eval(state_actual(i,:),L,W,R);
%     [observ_actual(i,:),H] = get_H(state_actual(i,:),L,W,1,R);
    % Time Update
    state_predict_temp = F*state_predict(i-1,:)';
    cov_t1 = F*cov_t*F' + Q;
    
    % residual
    residual(i-1,:) = (observ_actual(i,:)' - H*state_predict_temp)';
    
    % Kalman Gain
    K = cov_t1*H'/(H*cov_t1*H' + R);
    
    % Observation update
    state_predict(i-1,:) = (state_predict_temp + K*residual(i-1,:)')';
    cov_t = cov_t1 - K*H*cov_t1;
    
end

figure(1)
plot(state_predict(:,1),state_predict(:,2));
hold;
plot(state_actual(:,1),state_actual(:,2));

axis([0 750 0 500]);



function [obs,H_t] = get_H(x,L,W,flag_noise,R)
x0 = x(1);
y0 = x(2);
theta0 = normalize_theta(x(3));

% syms x0 y0 theta0;
% theta0

%Special cases
if theta0==0 || theta0 == -pi || theta0 == pi/2 || theta0 == -pi/2
    if theta0 == 0
        obs = [L-x0;y0;theta0];
        h = [L-x0;y0;theta0];
        H_t = [-1 0 0;0 1 0; 0 0 1];
    elseif theta0 == -pi
        obs = [x0;W-y0;theta0];
        h = [x0;W-y0;theta0];
        H_t = [1 0 0;0 -1 0; 0 0 1];
    elseif theta0 == pi/2
        obs = [W-y0;L-x0;theta0];
        h = [W-y0;L-x0;theta0];
        H_t = [0 -1 0;-1 0 0; 0 0 1];
    elseif theta0 == -pi/2
        obs = [y0;x0;theta0];
        h = [y0;x0;theta0];
        H_t = [0,1,0;1,0,0;0,0,1];
    end
    
%     H = jacobian(h,[x0;y0;theta0]);
%     H_t = vpa(subs(H,[x0,y0,theta0],[x0,y0,theta0]));
    
else    %Four Qurduants
    obs = zeros(3,1);
    if theta0>0 && theta0 <pi/2
        pf = find_nearst(x,L,W,[L,W]);
        pr = find_nearst([x0;y0;normalize_theta(theta0-pi/2)],L,W,[L,0]);
    elseif theta0>-pi/2 && theta0 <0
        pf = find_nearst(x,L,W,[L,0]);
        pr = find_nearst([x0;y0;normalize_theta(theta0-pi/2)],L,W,[0,0]);
        
    elseif theta0>-pi && theta0 <-pi/2
        pf = find_nearst(x,L,W,[0,0]);
        pr = find_nearst([x0;y0;normalize_theta(theta0-pi/2)],L,W,[0,W]);
        
    elseif theta0>pi/2 && theta0 <pi
        pf = find_nearst(x,L,W,[0,W]);
        pr = find_nearst([x0;y0;normalize_theta(theta0-pi/2)],L,W,[L,W]);
    end
    
    obs(1) = sqrt((pf(1) - x0)^2 + (pf(2) - y0)^2);
    obs(2) = sqrt((pr(1) - x0)^2 + (pr(2) - y0)^2);
    obs(3) = theta0;
    
%     h = [sqrt((pf(1) - x0)^2 + (pf(2) - y0)^2);sqrt((pr(1) - x0)^2 + (pr(2) - y0)^2);theta0];
    H_t = [ -(2*pf(1) - 2*x0)/(2*((pf(1) - x0)^2 + (pf(2) - y0)^2)^(1/2)), -(2*pf(2) - 2*y0)/(2*((pf(1) - x0)^2 + (pf(2) - y0)^2)^(1/2)), 0; ...
         -(2*pr(1) - 2*x0)/(2*((pr(1) - x0)^2 + (pr(2) - y0)^2)^(1/2)), -(2*pr(2) - 2*y0)/(2*((pr(1) - x0)^2 + (pr(2) - y0)^2)^(1/2)), 0; ...
                                                               0,                                                       0, 1];  
end

if flag_noise == 1
    obs = obs + sqrt(R) * normrnd(0,1,3,1);
end

end


function y = find_nearst(x,L,W,edges)
%edges: Horizaontal and Vertical edge boundaries
x0 = x(1);
y0 = x(2);
theta0 = x(3);

x1 = x0 + (edges(2) - y0)/tan(theta0);
y1 = y0 + (edges(1) - x0) * tan(theta0);


if y1 >= 0 && y1 <= W %On the vertical edge
    y = [edges(1);y1];
else %On the horizontal edge
    y = [x1;edges(2)];
end
end
function xnew = normalize_theta(x)
xnew = mod(x,2*pi);
if xnew > pi
    xnew = xnew - 2*pi;
end
end
