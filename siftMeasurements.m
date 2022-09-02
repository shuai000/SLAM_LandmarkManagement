function [sk, sk_dist, index] = siftMeasurements(zk, xk, radius_threshold, platform_stateLength, num_landmarkParam)
%% Select candidate measurements candidate for the state update
% Inputs:   zk                      - radar measurements
%           xk                      - state vector
%           radius_threshold        - radius of the circle used for sifting
%           platform_stateLength    - length of the state vector for the platform
%           num_landmarkParam       - number of parameters representing each landmark
%
% Outputs:  sk                      - selected measurements
%           sk_dist                 - distances between selected measurements and landmarks
%           index                   - indices of selected measurements in zk

num_landmarks = (length(xk) - platform_stateLength)/2;      % Number of landmarks in the state vector
if num_landmarks == 0
    % If no landmarks return empty selected measurements
    sk = [];
    sk_dist = [];
    index = [];
else
    % Select the landmarks from the state vector
    landmarks = reshape(xk(platform_stateLength+1:end), num_landmarkParam, num_landmarks)'; 
    N = size(zk, 1);                                    % Number of measurements
    % Calculate locations of the measurements in global x-y coordinate
    measurement_xy = zeros(N, 2);
    for i = 1:N
        measurement_xy(i, 1) = xk(1) + zk(i, 1) * cos(zk(i, 2) + xk(3));   
        measurement_xy(i, 2) = xk(2) + zk(i, 1) * sin(zk(i, 2) + xk(3));
    end
    % Select the measurements which are close to an existing landmark
    selected = zeros(N,1);
    distances = zeros(N, num_landmarks);
    for i=1:num_landmarks
        % Calculate the Euclidean distance between the current landmark and the measurements
        difference_value = landmarks(i, 1:2) - measurement_xy;   
        distances(:, i) = sqrt(sum(difference_value.^2, 2));   
        % Add the distances less than the threshold to the selected indices
        selected = selected | (distances(:, i) < radius_threshold);
    end
    index = find(selected==1);                          % Select the index numbers of the measurements
    sk = zk(index, :);                                  % Select the corresponding measurements
    sk_dist = distances(index, :);                      % Select the corresponding distances in the xy coordinates
end
end