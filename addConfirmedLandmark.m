function [xa, Pa, landmarks] = addConfirmedLandmark(xk, Pk, landmarks, ck, ID_No, R, num_landmarkParam, k, M)
%% Add a new landmark and state augmentation (see Appendix C of the paper)
% Inputs:   xk                  - state vector
%           Pk                  - state covariances
%           landmarks           - current confirmed landmarks structure
%           ck                  - cluster center
%           ID_No               - ID number for the new landmark
%           R                   - radar measurement noise covariance
%           num_landmarkParam   - number of parameters representing each landmark
%           k                   - current time index
%           M                   - length of the MN logic window
%
% Outputs:  xa                  - augmented state vector
%           Pa                  - augmented state covariance
%           landmarks           - updated confirmed landmarks structure

N = length(xk);                                     % Length of the state vector
psi = xk(3) + ck(2);                                % Angle from the platform heading and the azimuth of the landmark measurement
rk = ck(1);                                         % Range of the landmark measurement

% Calculate the Jacobian J1
J1 = zeros(N+num_landmarkParam, N);
for i=1:N
    J1(i,i) = 1;
end
J1(N+1, 1) = 1;
J1(N+1, 3) = -rk*sin(psi);
J1(N+2, 2) = 1;
J1(N+2, 3) = rk*cos(psi);

% Caclulate the Jacobian J2
J2 = zeros(N+num_landmarkParam, 2);
J2(N+1, 1) = cos(psi);
J2(N+1, 2) = -rk * sin(psi);
J2(N+2, 1) = sin(psi);
J2(N+2, 2) = rk * cos(psi);

% State augmentation
xa = [xk; ck(1); ck(2)];                            % Add the new landmark to the state vector
Pa = J1 * Pk * J1' + J2 * R * J2';                  % Update the state covariance

% Add the landmark to the confirmed landmarks structure
landmarks(ID_No).ID = ID_No;                        % Set the landmark ID number
landmarks(ID_No).createTime = k;                    % Set the creation time
landmarks(ID_No).removeTime = -1;                   % Set the remove time
landmarks(ID_No).position = ck;                     % Set the current location of the landmark  
landmarks(ID_No).association = [1, zeros(1, M-1)];  % Initialize the association vector
landmarks(ID_No).withinRange = [1, zeros(1, M-1)];  % Initialize the within range vector
end