function [landmark_index, min_H, min_hx] = associateMeasurements(sk, sk_dist, xk, Pk, R, beta, Ns, platform_stateLength, num_landmarkParam)
%% Apply a finer association based on the sifting results and a log likelihood based distance metric
% Inputs:   sk                      - candidate measurement for association    
%           sk_dist                 - distances between candidate measurement and landmarks
%           xk                      - state vector
%           pk                      - state covariance
%           R                       - measurement noise
%           beta                    - sifting threshold
%           Ns                      - number of landmarks to use in the sifting
%           platform_stateLength    - length of the state vector for the platform
%           num_landmarkParam       - number of parameters representing each landmark
%
% Outputs:  landmark_index          - index of the landmark the measurement is to be associated with
%           min_H                   - Jacobian of the measurement model for the selected landmark
%           min_hx                  - landmark position in range and azimuth

num_landmarks = (length(xk) - platform_stateLength)/num_landmarkParam;                  % Number of landmarks in the state vector
Ns = min(Ns, num_landmarks);                                                            % Adjust Ns if there are not Ns landmarks
landmarks = reshape(xk(platform_stateLength+1:end), num_landmarkParam, num_landmarks)'; % Select the landmarks from the state vector
[~, sorted_index] = sort(sk_dist);                                                      % Sort the Euclidian distance between the measurement and the landmarks

% Calculate the log distance for the Ns landmarks closest to the selected measurement
logD = zeros(1,Ns);
H = zeros(Ns, num_landmarkParam, length(xk));
h_x = zeros(Ns, 2);
for i=1:Ns
    j = sorted_index(i);                                                                % Select the index of the landmark
    [logD(i), H(i,:,:), h_x(i,:)] = calcLogDistance(sk(1:2), xk, Pk, landmarks(j, 1:2), R, j, platform_stateLength, num_landmarkParam);
end
% Find the smallest log distance
[min_logD, min_index] = min(logD);

if min_logD < beta
    % If the smallest log distance is less than the threshold set the index of the landmark it corresponds to
    landmark_index = sorted_index(min_index);
    min_H = squeeze(H(min_index,:,:));
    min_hx = h_x(min_index,:);
else
    % If no log distances are less than the threshold the measurement is not associated with a landmark
    landmark_index = -1; 
    min_H = [];
    min_hx = [];
end

end