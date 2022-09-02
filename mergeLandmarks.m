function [state, id] = mergeLandmarks(state, k, id_list, threshold_xy, threshold_p, platform_stateLength, num_landmarkParam)
%% Function to merge two landmarks if they are very close to each other
% Inputs:   state                   - system state
%           k                       - current time index
%           id_list                 - list of landmark IDs
%           threshold_xy            - threshold for how close the landmarks should be in either x or y to be merged
%           threshold_p             - threshold for how close the landmarks should be in the Euclidean distance to be merged
%           platform_stateLength    - length of the state vector for the platform
%           num_landmarkParam       - number of parameters representing each landmark
%
% Outputs:  state                   - system state
%           id                      - ids of the landmarks which are removed

% Get the positions of the landmarks 
Lk_position =  reshape(state(k).x_update(platform_stateLength+1:end), [num_landmarkParam, length(state(k).x_update(platform_stateLength+1:end))/num_landmarkParam])';
N = size(Lk_position, 1);   % Calculate the number of landmarks

% For each landmark calculate the distance in x, distance in y and Euclidean distance to every other landmark
dist = zeros((N*(N-1))/2,5);
idx = 1;
for i = 1:N
    for j = i+1:N
        x_dist = abs(Lk_position(i,1) - Lk_position(j,1));
        y_dist = abs(Lk_position(i,2) - Lk_position(j,2));
        dist(idx,:) = [i, j, x_dist, y_dist, sqrt(x_dist^2 + y_dist^2)];
        idx = idx+1;
    end
end

dist = dist(dist(:,5)<threshold_p,:);   % Select those rows where the Euclidean distance is below the threshold
[~, idx] = sort(dist(:,5));             % Sort rows by the Euclidean distance
dist = dist(idx,:);

% Check if any landmarks are eligible for merging
merge_indicator = -1;
for i = 1:length(idx)
    % Check if landmarks are less than threshold_xy apart in either x or y
    if dist(1,3) <= threshold_xy || dist(1,4) <= threshold_xy
        merge_indicator = dist(1,2);    % Identify the landmark to be merged
        break;
    end
end

% Merge the landmarks
id = [];
if merge_indicator ~= -1
    start_index = platform_stateLength+1 + num_landmarkParam*(merge_indicator-1);   % Find the start location of the landmark in the state
    end_index = start_index + (num_landmarkParam-1);                                % Find the end location of the landmark in the state
    state(k).x_update(start_index:end_index) = [];                                  % Remove the landmark from the state update
    state(k).cov_update(start_index:end_index, :) = [];                             % Remove the landmark from the covariance update
    state(k).cov_update(:, start_index:end_index) = [];
    id = id_list(merge_indicator);                                                  % Add the landmark id to the list of removed landmarks
end

end