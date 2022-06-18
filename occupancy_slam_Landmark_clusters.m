%%% Code for paper "Landmark management in the application of RadarSLAM"
%%% Author: Shuai SUN
%%% Co-author: Chris Gilliam
%%% Email: shuai.sun@dlmu.edu.cn
%%% Date: 16/06/2022

clc; clear all; close all;

% Define the landmark(vehicle) position and shape in 2D
landmark_loading;
% Provide Algorithm parameters, see Table I in the paper
parameter_loading;
% Simulation robot motion data and radar measurement data
data_generation;
% Initialization of the robot EKF-SLAM
initialization;

for t=1:T
    struct_state(t).x_update = struct_state(t).x_predict;
    struct_state(t).cov_update = struct_state(t).cov_predict;
    if ~isempty(measurement_all{t})
        %% EKF-Update
        % Measurement association
        echo.z = measurement_all{t};
        % obtain the measurement candidate to be associated
        [sk, sk_index] = sifting(echo.z, struct_state(t).x_update, thresholds.sifting_ridus);
        associated_index = []; % record the index of sk that is associated with existing landmarks
        if ~isempty(sk)
            recorded_measurement = [];
            % Association
            for i=1:length(sk_index)
                landmark_index = sift_association(sk(i, :), struct_state(t).x_update, struct_state(t).cov_update, sensor.R_radar_filter, thresholds.beta, thresholds.Ns);
                if landmark_index ~= -1 % the ith sk is associated with a landmark whose index is landmark_index
                    recorded_measurement = [recorded_measurement, landmark_index];
                    % Measurement update (EKF)
                    [ struct_state(t).x_update, struct_state(t).cov_update] = ekf_update(struct_state(t).x_update, struct_state(t).cov_update, sk(i, 1:2), landmark_index, sensor.R_radar_filter);
                    associated_index = [associated_index, sk_index(i)];
                end
            end
            % Record association information using linked list
            associated = id_list(unique(recorded_measurement));
            not_associated = setdiff(id_list, associated);
            
            for j=1:length(associated)
                child_data.ID = associated(j);
                child_data.association = 1;
                child_data.withinRange = 1;
                temp_node = dlnode(child_data);
                for jj = 1:counter_landmark
                    if ~isempty(landmark_list_cell{jj}) && landmark_list_cell{jj}.Data.ID == child_data.ID
                        temp = landmark_list_cell{jj};
                        while 1
                            if isempty(temp.Next)
                                temp_node.insertAfter(temp);
                                break;
                            end
                            temp = temp.Next;
                        end
                        break;
                    end
                end
            end
            
            for j=1:length(not_associated)
                child_data.ID = not_associated(j);
                child_data.association = 0;
                for jj = 1:counter_landmark
                    if ~isempty(landmark_list_cell{jj}) && landmark_list_cell{jj}.Data.ID == child_data.ID
                        child_data.withinRange = is_withinRange(struct_state(t).x_update(1:2),landmark_list_cell{jj}.Data.position, sensor.R_MAX);
                        temp_node = dlnode(child_data);
                        temp = landmark_list_cell{jj};
                        while 1
                            if isempty(temp.Next)
                                temp_node.insertAfter(temp);
                                break;
                            end
                            temp = temp.Next;
                        end
                        break;
                    end
                end
            end
            
            %% landmark deleting logic
            for m = 1:counter_landmark
                if ~isempty(landmark_list_cell{m})
                    [end_node, num] = find_end_node(landmark_list_cell{m});
                    if num >= mn_m
                        withinRangeCounter = 0;
                        associationCounter = 0;
                        for n = 1:mn_m
                            % Compute Counter
                            withinRangeCounter = withinRangeCounter + end_node.Data.withinRange;
                            associationCounter = associationCounter + end_node.Data.association;
                            end_node = end_node.Prev;
                        end
                        if withinRangeCounter == mn_m
                            if associationCounter <= mn_n
                                % execute delete
                                id = landmark_list_cell{m}.Data.ID; % this id needs to be deleted
                                id_index = find(id_list == id); % the id_index-th term is the data
                                
                                from_index = Lk_startIndex + (id_index-1)*2;
                                % a) delete state and cov
                                struct_state(t).x_update(from_index:from_index+1) = [];
                                struct_state(t).cov_update([from_index, from_index+1], :) = [];
                                struct_state(t).cov_update(:, [from_index, from_index+1]) = [];
                                % b) delete id_list
                                id_list(id_index) = [];
                                counter_id = counter_id - 1;
                                % c) delete landmark cell list
                                landmark_list_cell{id} = [];
                            end
                        end
                    end
                end
            end
        end
        
        % New landmark detection and inclusion
        inclusion_candidate = echo.z;
        inclusion_candidate(sk_index, :) = []; % delelte measurements that have been associated with existing landmarks
        if isempty(inclusion_candidate)
            clusters = [];
        else
            clusters = cluster_dbscan(inclusion_candidate, struct_state(t).x_update, struct_state(t).cov_update, thresholds.cluster_ridus, sensor.R_radar_filter, thresholds.Na, thresholds.min_cluster_points);
        end
        % global temp_head;
        if t==1  % build temp_head
            ii = 0;
            for ic=1:length(clusters)
                if min(clusters(ic).logD) > thresholds.alpha
                    
                    if size(clusters(ic).z, 1) >= thresholds.point_threshold % confirmed
                        z_include = clusters(ic).ck(1:2);
                        [struct_state(t).x_update, struct_state(t).cov_update, lx, ly] = state_augmentation(struct_state(t).x_update, struct_state(t).cov_update, z_include, sensor.R_radar_filter);
                        % add to linked_list for landmark management
                        counter_landmark = counter_landmark + 1;
                        landmark_list_cell{counter_landmark} = dlnode(add_node_to_landmark(lx, ly, t, counter_landmark));
                        counter_id = counter_id + 1;
                        id_list(counter_id) = counter_landmark;
                    else
                        ii = ii + 1;
                        temp_head(t).centerinfo(ii, :) = clusters(ic).ck;
                        temp_head(t).association_counter(ii) = 1;
                        temp_head(t).window_length(ii) = 1;
                        temp_head(t).detection_points(ii) = size(clusters(ic).z, 1);
                        temp_head(t).xy(ii, :) = mean(clusters(ic).xy, 1);
                        
                        have_temp_head = 1;
                    end
                end
            end
            
        else  % association and landmark head management
            % 1> association
            if have_temp_head
                [temp_head(t), clusters_inclusion] = candidate_landmark_association(temp_head(t-1), clusters, struct_state(t).x_update, thresholds.association_ridus);
                % 2> confirmation and inclusion
                Lk_temp = reshape(struct_state(t).x_update(Lk_startIndex:end), [2, length(struct_state(t).x_update(Lk_startIndex:end))/2])';
                detection_temp = [];
                deleting_index = [];
                for ih=1:length(temp_head(t).association_counter)
                    if temp_head(t).association_counter(ih) >= MN_N || temp_head(t).detection_points(ih) >= thresholds.point_threshold % confirmed
                        z_include = temp_head(t).centerinfo(ih, 1:2);
                        [struct_state(t).x_update, struct_state(t).cov_update, lx, ly] = state_augmentation(struct_state(t).x_update, struct_state(t).cov_update, z_include, sensor.R_radar_filter);
                        detection_temp = [detection_temp; lx, ly];
                        deleting_index = [deleting_index, ih];
                        % add to linked_list for landmark management
                        counter_landmark = counter_landmark + 1;
                        landmark_list_cell{counter_landmark} = dlnode(add_node_to_landmark(lx, ly, t, counter_landmark));
                        counter_id = counter_id + 1;
                        id_list(counter_id) = counter_landmark;
                    end
                end
                % deleting confirmed head
                temp_head(t) = delete_candidate_landmarks(temp_head(t), deleting_index);
                Lk_detected{t} = detection_temp;
                % 3> window length +1
                temp_head(t).window_length = temp_head(t).window_length + 1;
                % 4> deleting windowed out head
                deleting_index = find_to_delete(temp_head(t), MN_M, MN_N);
                temp_head(t) = delete_candidate_landmarks(temp_head(t), deleting_index);
                % 5> add non-associated cluster to head
                for ic=1:length(clusters_inclusion)
                    if min(clusters_inclusion(ic).logD) > thresholds.alpha
                        
                        if size(clusters_inclusion(ic).z, 1) >= thresholds.point_threshold % confirmed
                            z_include = clusters_inclusion(ic).ck(1:2);
                            [struct_state(t).x_update, struct_state(t).cov_update, lx, ly] = state_augmentation(struct_state(t).x_update, struct_state(t).cov_update, z_include, sensor.R_radar_filter);
                            % add to linked_list for landmark management
                            counter_landmark = counter_landmark + 1;
                            landmark_list_cell{counter_landmark} = dlnode(add_node_to_landmark(lx, ly, t, counter_landmark));
                            counter_id = counter_id + 1;
                            id_list(counter_id) = counter_landmark;
                        else
                            temp_head(t).centerinfo(end+1, :) = clusters_inclusion(ic).ck;
                            temp_head(t).association_counter(end+1) = 1;
                            temp_head(t).window_length(end+1) =1;
                            temp_head(t).detection_points(end+1) = size(clusters_inclusion(ic).z, 1);
                            temp_head(t).xy(end+1,:) = mean(clusters_inclusion(ic).xy, 1);
                        end
                    end
                end
            else
                % build temp_head （used if there is no clusters at the
                % beginning)
                ii = 0;
                for ic=1:length(clusters)
                    if min(clusters(ic).logD) > thresholds.alpha
                        ii = ii + 1;
                        temp_head(t).centerinfo(ii, :) = mean(clusters(ic).z, 1);
                        temp_head(t).association_counter(ii) = 1;
                        temp_head(t).window_length(ii) = 1;
                        temp_head(t).detection_points(ii) = size(clusters(ic).z, 1);
                        temp_head(t).xy(ii, :) = mean(clusters(ic).xy, 1);
                        temp_head(t).time(ii) = t;
                        
                        have_temp_head = 1;
                    end
                end
            end
        end
    end
    
    %% Landmark Merge
    [struct_state, removed_ids] = landmark_merge(struct_state, t, id_list, 0.8, 3);
    % remove the merged ids
    for ir = 1:length(removed_ids)
        id_to_remove = removed_ids(ir);
        % b) delete id_list
        id_list(find(id_list == id_to_remove)) = [];
        counter_id = counter_id - 1;
        % c) delete landmark cell list
        for mr=1:counter_landmark
            if landmark_list_cell{mr}.Data.ID == id_to_remove
                landmark_list_cell{mr}.Data.removeTime = t;
            end
        end
    end
    
    % State prediction
    if t<T
        [struct_state(t+1).x_predict, struct_state(t+1).cov_predict] = ekf_prediction(struct_state(t).x_update, struct_state(t).cov_update, odo_reading(:,t), sensor.R_odometer, Q);
    end
    
    clf
    plot(theta_vector(1, 1:t), theta_vector(2, 1:t), 'k.', 'MarkerSize', 3); hold on;
    
    for i=1:size(landmarks_cell{t},1)
        rectangle('Position',landmarks_cell{t}(i,:), 'LineWidth',0.8, 'Curvature',0.3);
    end
    
    drawcircle('Center',[theta_vector(1, t), theta_vector(2, t)],'Radius',sensor.R_MAX,'StripeColor','green',  'LineWidth', .1, 'Visible', 'on');
    
    %%% plot the EKF-SLAM result
    plot(clutter_point{t}(:, 1), clutter_point{t}(:, 2), 'c*'); hold on;
    if ~isempty(reflection_point{t})
        plot(reflection_point{t}(:, 1), reflection_point{t}(:, 2), 'k.'); hold on;
    end
    Lk_position =  reshape(struct_state(t).x_update(Lk_startIndex:end), [2, length(struct_state(t).x_update(Lk_startIndex:end))/2])';
    plot(Lk_position(:, 1),Lk_position(:, 2), 'md', 'MarkerSize', 5); hold on;
    plot(struct_state(t).x_update(1), struct_state(t).x_update(2), 'r.', 'Markersize', 12);
    xlim([-20, 40]);
    ylim([-20, 40]);
    xlabel('X(m)');
    ylabel('Y(m)');
    grid off
    axis equal
    pause(.001);
end
