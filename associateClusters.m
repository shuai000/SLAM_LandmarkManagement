function [landmarks, clusters] = associateClusters(landmarks, clusters, xk, threshold)
%% Associate clusters with unconfirmed landmarks 
% Inputs:   landmarks   - unconfirmed landmarks from the previous time step
%           clusters    - clusters from the current measurements
%           xk          - current state vector
%           threshold   - threshold for the radius of the circle used to determine if landmarks are to be associated
%
% Outputs:  landmarks   - updated unconfirmed landmarks 
%           clusters    - clusters that have not been associated with a landmark

M = length(landmarks.association_counter);  % Number of unconfirmed landmarks at previous time step
N = length(clusters);                       % Number of clusters from current measurements

% Calculate whether each landmark should be associated with each cluster
distances = zeros(M*N, 3);
for i=1:N
    % Calculate distances between cluster and each landmark
    distances((i-1)*M+1:i*M,:) = [i*ones(M,1), (1:M)', sqrt((landmarks.centerinfo(:,1) - clusters(i).ck(1)).^2 + (landmarks.centerinfo(:,2) - clusters(i).ck(2)).^2)];
end
% Remove entries with distances greater than the threshold
distances = distances(distances(:,3) < threshold, :);                       

clusters_to_delete = zeros(1,N);
while ~isempty(distances)
    [~, i] = min(distances(:,3));                                           % Find the entry with the minimum distance
    c_index = distances(i,1);                                               % Get the index of the cluster with the min distance
    l_index = distances(i,2);                                               % Get the index of the landmark with the min distance
    % Update the landmark based on the associated cluster
    landmarks.association_counter(l_index) = landmarks.association_counter(l_index) + 1;
    landmarks.centerinfo(l_index, :) = clusters(c_index).ck;                % Update the landmark center
    landmarks.detection_points(l_index) = size(clusters(c_index).z, 1);     % Update the number of detection points
    clusters_to_delete(c_index) = 1;                                        % Add the cluster to the clusters to be removed
    % Clusters can only be associated with one landmark and vice versa
    distances((distances(:,1)==c_index | distances(:,2)==l_index),:) = [];  % Remove any entries with the same cluster or landmark  
end
% Remove clusters which have been associated with a landmark
clusters(clusters_to_delete==1) = [];
end