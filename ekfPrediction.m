function [X, P] = ekfPrediction(xk, Pk, uk, U, Q0, dt)
%% State prediction based on EKF filter and motion model
% Inputs:   xk  - state vector
%           Pk  - state covariance
%           uk  - odometer reading [velocity; yaw rate] 
%           U   - covariance matrix for odometer reading 
%           Q   - covariance matrix for the platform process noise
%           dt  - time interval
%
% Outputs:  X   - mean of the predicted state
%           P   - covariance of the predicted state

X = xk;                     % Initialize the predicted state
N = length(xk);             % Calculate the size of the state
theta = xk(3);              % Platform heading 
v = uk(1);                  % Velocity
psi = uk(2);                % Yaw rate
Q = zeros(N, N);            % Set the process noise for the state
Q(1:3, 1:3) = Q0;   

A = sin(theta + dt*psi/2);  % Calculate the angular updates
B = cos(theta + dt*psi/2);

% Apply the platform motion model to predict platform pose
X(1) = xk(1) + v * dt * B;
X(2) = xk(2) + v * dt * A;
X(3) = wrapTo2Pi(theta + psi*dt); 

% Calculate Jacobian w.r.t. state vector
Fx = eye(N);
Fx(1,3) = -dt * v * A;
Fx(2,3) = dt * v * B; 

% Caclulate Jacobin w.r.t the control input
Fu = zeros(N, 2);
Fu(1,1) = dt * B; 
Fu(1,2) = -(dt^2)/2 * v * A;
Fu(2,1) = dt * A; 
Fu(2,2) = (dt^2)/2 * v * B;
Fu(3,2) = 1;

% Predicted update of the state covariance
P = Fx * Pk * Fx' + Fu * U * Fu' + Q;

end