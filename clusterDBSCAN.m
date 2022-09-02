function data_cluster = clusterDBSCAN(z, xk, Pk, threshold, R, Na, min_np, platform_stateLength, num_landmarkParam)
%% Cluster the measurements using DBSCAN
% Inputs:   z                       - measurement vector [range, azimuth, SNR]
%           xk                      - state vector
%           Pk                      - state covariance
%           threshold               - threshold for the DBSCAN neighborhood radius
%           R                       - radar measurement noise
%           Na                      - number of landmarks to test for association
%           min_np                  - minimum number of points to be considered a cluster
%           platform_stateLength    - length of the state vector for the platform
%           num_landmarkParam       - number of parameters representing each landmark
%
% Output:   data_cluster            - struct that has the following fields:
%                                       z: measurements in polar coordinate for each cluster member
%                                       ck: xy coordinate of the measurement in the cluster with the largest SNR
%                                       dist: distance of ck w.r.t. each existing landmark

N = size(z, 1);                                                                             % Number of measurements
num_landmarks = (length(xk) - platform_stateLength)/2;                                      % Number of landmarks
landmarks = reshape(xk(platform_stateLength+1:end), num_landmarkParam, num_landmarks)';     % Landmark locations

% Convert measurements into global x-y coordinate
data = zeros(N, 2);     
for i = 1:N
    data(i, 1) = xk(1) + z(i, 1) * cos(z(i, 2) + xk(3));
    data(i, 2) = xk(2) + z(i, 1) * sin(z(i, 2) + xk(3));
end

% Perform DBSCAN clustering
idx = dbscan(data, threshold, min_np);

if max(idx)==-1
    data_cluster = [];                                                          % If no clusters create empty cluster structure
else
    data_cluster(max(idx)) = struct('z',[], 'ck', [], 'logD', []);              % Otherwise create a structure the size of the number of clusters
    for i=1:max(idx)
        data_cluster(i).z = z(idx==i,:);                                        % Store the measurements in the structure
        [~, max_index] = max(data_cluster(i).z(:, 3));                          % Find the measurement in the cluster with the highest SNR
        tmp = data(idx==i,:);
        data_cluster(i).ck = tmp(max_index, :);                                 % Set the center of the cluster to the xy coordinate of the highest SNR measurement
        
        if num_landmarks > 0
            % Calculate the Eudician distance between the cluster center and the landmarks
            dist_vector = sqrt(sum((data_cluster(i).ck - landmarks(:,1:2)).^2, 2));
            [~, sorted_index] = sort(dist_vector);                              % Sort the distances
            Na = min(Na, length(sorted_index));                                 % Set the number of landmarks to test
            % Select the closest Nc landmarks and compute log distance (computation issue)
            logD = Inf(1,Na);
            for iu=1:Na
                logD(iu) = calcLogDistance(data_cluster(i).z(max_index,1:2), xk, Pk, landmarks(sorted_index(iu), 1:2), R, sorted_index(iu), platform_stateLength, num_landmarkParam);
            end
            data_cluster(i).logD = logD;                                        % Store the log distances from the ith cluster to the nearest Nc landmarks
        else
            data_cluster(i).logD = Inf;                                         % If no landmarks set the log distance
        end
    end
end

end