%% Code for paper "Landmark management in the Application of Radar SLAM"
% Author: Shuai SUN
% Co-author: Chris Gilliam
% Email: shuai.sun@dlmu.edu.cn
% Date: 16/06/2022
%
% Usage: This script can be used to replicate the simulations in the paper,
% landmark locations and algorithm parameters can be changed the relevant
% scripts

clc; clearvars; close all;

% Load the algorithm parameters
loadParameters;

% Simulate platform motion data and radar measurement data
[measurement_all, odo_reading, platform_stateVector, landmarks, reflection_point, clutter_point] = generateData(sensor, range, T, dt, lambda);
% Initialize the EKF-SLAM
initialize;

for k=1:T
    state(k).x_update = state(k).x_predict;             %#ok<*SAGROW> % Update state
    state(k).cov_update = state(k).cov_predict;         % Update covariance
    if ~isempty(measurement_all{k})
        %% EKF-Update
        % Obtain the candidate measurements for association
        [sk, sk_dist, sk_index] = siftMeasurements(measurement_all{k}, state(k).x_update, thresholds.sifting_radius, platform_stateLength, num_landmarkParam);
        if ~isempty(sk)
            observed_landmarks = zeros(1,id_list_count);
            for i=1:length(sk_index)
                % Perform association
                [landmark_index, H, h_x] = associateMeasurements(sk(i, :), sk_dist(i,:), state(k).x_update, state(k).cov_update, filter.R_radar, thresholds.beta, thresholds.Ns, platform_stateLength, num_landmarkParam);
                if landmark_index ~= -1         
                    % The ith measurement in sk is associated with the landmark with index landmark_index
                    observed_landmarks(landmark_index) = 1;
                    % EKF measurement update
                    [state(k).x_update, state(k).cov_update] = ekfUpdate(state(k).x_update, state(k).cov_update, sk(i, 1:2), H, filter.R_radar, h_x);
                end
            end
            % Identify which landmarks have been associated with measurements
            associated = id_list(observed_landmarks==1);
            not_associated = id_list(observed_landmarks==0);
            
            for i=1:length(associated)
                % Find the location of the landmark in the state vector
                landmark_index = find(id_list==associated(i));
                x_index = platform_stateLength + 1 + (landmark_index-1)*num_landmarkParam;
                % Update the landmark
                confirmed_landmarks(associated(i)).position = state(k).x_update(x_index:x_index+1)';
                confirmed_landmarks(associated(i)).association = [1, confirmed_landmarks(associated(i)).association(1:end-1)];
                confirmed_landmarks(associated(i)).withinRange = [1, confirmed_landmarks(associated(i)).withinRange(1:end-1)];
            end
            
            for i=1:length(not_associated)
                % Check whether the landmark is within detection range
                detectRange = withinRange(state(k).x_update(1:2), confirmed_landmarks(not_associated(i)).position, sensor.max_range);
                % Update the landmark
                confirmed_landmarks(not_associated(i)).association = [0, confirmed_landmarks(not_associated(i)).association(1:end-1)];
                confirmed_landmarks(not_associated(i)).withinRange = [detectRange, confirmed_landmarks(not_associated(i)).withinRange(1:end-1)];
            end
            clear i j sk sk_dist landmark_index H h_x observed_landmarks detectRange  
            
            %% Landmark deleting
            for i = id_list
                if (k - MN_logic.M_deleting + 1) >= confirmed_landmarks(i).createTime
                    if sum(confirmed_landmarks(i).withinRange) == MN_logic.M_deleting
                        if sum(confirmed_landmarks(i).association) <= MN_logic.N_deleting
                            % If not associated with measurements at least N times execute delete
                            id_index = find(id_list==i);      % location of ID of landmark to be deleted
                            % Location of landmark in the state vector
                            start_index = platform_stateLength + 1 + (id_index-1)*num_landmarkParam;
                            end_index = start_index + num_landmarkParam - 1;
                            % Delete landmark form state and cov
                            state(k).x_update(start_index:end_index) = [];
                            state(k).cov_update(start_index:end_index, :) = [];
                            state(k).cov_update(:, start_index:end_index) = [];
                            % Delete landmark from the id_list
                            id_list(id_index) = [];
                            id_list_count = id_list_count - 1;
                            % Set the landmark remove time
                            confirmed_landmarks(i).removeTime = k;
                        end
                    end
                end
            end
            clear i j start_index end_index 
        end
        
        %% New landmark detection and inclusion
        % Select measurements which have not been associated with an existing landmark as candidates for clustering
        candidate_measurements = measurement_all{k};
        candidate_measurements(sk_index, :) = [];    
        if isempty(candidate_measurements)
            clusters = [];
        else
            % If there are candidate measurements cluster them using DBSCAN
            clusters = clusterDBSCAN(candidate_measurements, state(k).x_update, state(k).cov_update, thresholds.cluster_radius, filter.R_radar, thresholds.Na, thresholds.min_cluster_points, platform_stateLength, num_landmarkParam);
        end

        % Cluster association and unconfirmed landmark management
        if k ~= 1 && ~isempty(unconfirmed_landmarks(k-1).association_counter)
            % Associate clusters with existing unconfirmed landmarks and remove from clusters
            [unconfirmed_landmarks(k), clusters] = associateClusters(unconfirmed_landmarks(k-1), clusters, state(k).x_update, thresholds.association_radius);

            % Check if any unconfirmed landmarks can now be confirmed
            deleting_index = zeros(1, unconfirmed_count);
            for i = 1:unconfirmed_count
                if unconfirmed_landmarks(k).association_counter(i) >= MN_logic.N_association || unconfirmed_landmarks(k).detection_points(i) >= thresholds.point_threshold 
                    % If either the unconfirmed landmark has been associated with measurements at least N times or has more than the threshold for the number of measurements it is confirmed
                    landmark_ID_No = landmark_ID_No + 1;                                                            % Increase the landmark counter
                    id_list_count = id_list_count + 1;                                                              % Increase the current landmark ID list counter
                    id_list(id_list_count) = landmark_ID_No;                                                        % Add the landmark ID to the ID list
                    [state(k).x_update, state(k).cov_update, confirmed_landmarks] = addConfirmedLandmark(state(k).x_update, state(k).cov_update, confirmed_landmarks, unconfirmed_landmarks(k).centerinfo(i, 1:2), landmark_ID_No, filter.R_radar, num_landmarkParam, k, MN_logic.M_deleting);
                    deleting_index(i) = 1;                                                                          % Add index to list of unconfirmed landmarks to be deleted
                end
            end
            % Delete unconfirmed landmarks that have now been confirmed or according to MN logic
            [unconfirmed_landmarks(k), unconfirmed_count] = deleteUnconfirmedLandmarks(unconfirmed_landmarks(k), deleting_index, unconfirmed_count, MN_logic.M_association, MN_logic.N_association);
        end
            
        % Add any clusters which have not been associated with existing landmarks as new landmarks
        for i = 1:length(clusters)
            if min(clusters(i).logD) > thresholds.alpha
                % If cluster is sufficiently far from existing clusters check its size
                if size(clusters(i).z, 1) >= thresholds.point_threshold 
                    % If the number of measurements in the cluster is large enough it is automatically confirmed as a landmark
                    landmark_ID_No = landmark_ID_No + 1;                                                        % Increase the landmark counter
                    id_list_count = id_list_count + 1;                                                          % Increase the current landmark ID list counter
                    id_list(id_list_count) = landmark_ID_No;                                                    % Add the landmark ID to the ID list
                    [state(k).x_update, state(k).cov_update, confirmed_landmarks] = addConfirmedLandmark(state(k).x_update, state(k).cov_update, confirmed_landmarks, clusters(i).ck, landmark_ID_No, filter.R_radar, num_landmarkParam, k, MN_logic.M_deleting);
                else
                    % If not sufficiently large enough to be automatically confirmed add cluster to unconfirmed landmarks
                    unconfirmed_count = unconfirmed_count + 1;                                                  % Increase the number of unconfirmed landmarks stored
                    unconfirmed_landmarks(k).centerinfo(unconfirmed_count, :) = clusters(i).ck;                 % Add the center of the cluster as the location for the new unconfirmed landmark
                    unconfirmed_landmarks(k).association_counter(unconfirmed_count) = 1;                        % Set the association counter for the new unconfirmed landmark
                    unconfirmed_landmarks(k).window_length(unconfirmed_count) = 1;                              % Set the window length for the new unconfirmed landmark
                    unconfirmed_landmarks(k).detection_points(unconfirmed_count) = size(clusters(i).z, 1);      % Add the number of radar measurements (detections) in the cluster
                end
            end
        end
        clear i sk_index candidate_measurements clusters deleting_index
    end
    
    %% Merge Landmarks
    [state, removed_id] = mergeLandmarks(state, k, id_list, thresholds.merge_dist, thresholds.merge_radius, platform_stateLength, num_landmarkParam);
    if ~isempty(removed_id)
        % Delete removed landmark from ID list
        id_list(id_list == removed_id) = [];
        id_list_count = id_list_count - 1;
        % Set the remove time of the landmark 
        confirmed_landmarks(removed_id).removeTime = k;
    end
    clear removed_id
    
    %% State prediction
    if k<T
        [state(k+1).x_predict, state(k+1).cov_predict] = ekfPrediction(state(k).x_update, state(k).cov_update, odo_reading(:,k), filter.U_odometer, filter.Q_process, dt);
    end
    
    %% Plot the EKF-SLAM result
    figure(1), clf, hold on;
    % Plot the true landmark positions
    for i=1:size(landmarks{k},1)
        rectangle('Position',landmarks{k}(i,:), 'LineWidth',0.8, 'Curvature',0.3);
    end
    % Adjust the figure
    xlim(range(1,:));
    ylim(range(2,:));
    xlabel('X(m)');
    ylabel('Y(m)');
    axis equal
    % Plot the true platform path
    platform_path = plot(platform_stateVector(1, 1:k), platform_stateVector(2, 1:k), 'k.', 'MarkerSize', 3);
    % Draw the region the radar sensor can currently detect
    sensor_circ = drawcircle('Center', [platform_stateVector(1, k), platform_stateVector(2, k)], 'Radius', sensor.max_range, 'LineWidth', .1, 'Visible', 'on', 'markerSize', .1);
    % Plot the clutter points and true landmark detections
    clutter_plot = plot(clutter_point{k}(:, 1), clutter_point{k}(:, 2), '*', 'color', [0.4660 0.6740 0.1880]);
    if ~isempty(reflection_point{k})
        reflection_plot = plot(reflection_point{k}(:, 1), reflection_point{k}(:, 2), '.', 'color', [0 0.4470 0.7410]);
    end
    % Plot the estimated landmarks
    Lk_position =  reshape(state(k).x_update(platform_stateLength+1:end), [num_landmarkParam, length(state(k).x_update(platform_stateLength+1:end))/num_landmarkParam])';
    Lk_plot = plot(Lk_position(:, 1), Lk_position(:, 2), 'd', 'MarkerSize', 5, 'color', [0.6350 0.0780 0.1840], 'MarkerFaceColor', [0.6350 0.0780 0.1840]);
    % Plot the estimated platform position
    platform_position = plot(state(k).x_update(1), state(k).x_update(2), 'm.', 'Markersize', 20);
    pause(.001);
end
