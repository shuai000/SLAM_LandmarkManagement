function [d, H, h_x] = calcLogDistance(zk, xk, Pk, Lk, R, j, platform_stateLength, num_landmarkParam)
%% Compute the distance between a given detection and a registered landmark
% Inputs:   zk                      - selected measurement 
%           xk                      - state vector
%           Pk                      - state covariance
%           Lk                      - landmark location
%           R                       - radar measurement noise
%           j                       - the jth landmark in xk
%           platform_stateLength    - length of the state vector for the platform
%           num_landmarkParam       - number of parameters representing each landmark
%
% Output:   d                       - log likelihood distance
%           H                       - Jacobian of the measurement model
%           h_x                     - landmark position in range and azimuth

% Select the current pose of the platform from the state vector
platform.x = xk(1); 
platform.y = xk(2); 
platform.theta = xk(3);
% Convert the landmark position to range and azimuth
[h_r, h_a] = radarMeasurement(platform, Lk, [0, 0]);
h_x = [h_r, h_a];

ek = (zk - h_x);                                                % Calculate residual
% Note that when the azimuth is close to pi, due to noise pertubation, abnormal situations may occur at residual computation
% I refined the residual in this way, please email me if you have a better solution (shuai.sun@dlmu.edu.cn), thank you.
if zk(2)>2*pi/3 && h_x(2)<-2*pi/3 
    % Prevent azimuth residual from being large if measurement is close to pi and landmark is close to -pi  
    ek(2) = zk(2) - (2*pi + h_x(2));                            % Add 2pi to landmark giving a small negative result
elseif zk(2)<-2*pi/3 && h_x(2)>2*pi/3
    % Prevent azimuth residual from being large if measurement is close to -pi and landmark is close to pi 
    ek(2) = 2*pi + zk(2) - h_x(2);                              % Add 2pi to measurement giving a small positive result
end

H = zeros(num_landmarkParam, length(xk));   % Initialize the Jacobian

% Get the location in the state vector of the landmark
j1 = platform_stateLength + (j-1)*num_landmarkParam + 1;       
j2 = platform_stateLength + (j-1)*num_landmarkParam + 2;

% Calculate the distances between the platform and the landmark
dx = xk(j1) - xk(1);
dy = xk(j2) - xk(2);
rk = sqrt(dx^2 + dy^2);

% Form the Jacobian
H(1,1) = -dx/rk; 
H(1,2) = -dy/rk; 
H(1,j1) = dx/rk; 
H(1,j2) = dy/rk;
H(2,1) = dy/(rk^2); 
H(2,2) = -dx/(rk^2); 
H(2,3) = -1;  % IMPORTANT DOUBLE CHECK THIS ITEM
H(2,j1) = -dy/(rk^2); 
H(2,j2) = dx/(rk^2);

S = H * Pk * H' + R;                                            % Residual covariance
% Calculate the likelihood
g = 1 / sqrt((2*pi)^2 * det(S));
ex = -0.5 * (ek/S) * ek';
d = -log(g * exp(ex));                                          % Calculate the distance as the negative log likelihood

end