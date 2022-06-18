%%% This script is to generate Radar SLAM simulation data
max_landmark_num = size(landmark_groundtruth_1,1);
landmarks_cell = cell(1, T);
t1 = 60;
for t=1:T
    if t<t1
        landmarks_cell{t} = landmark_groundtruth_1;
    else
        landmarks_cell{t} = landmark_groundtruth_2;
    end
end

% Control Input
u = zeros(2, T); % first row: velocity, second row: angular velocity
u(1, :) = 4;
u(2,10:100) = 2*dt;

% Odometer reading
% Apply the odometer model defined in Eq.(6)
odo_noise = mvnrnd([0, 0],sensor.R_odometer , T);
radar_noise = zeros(T, 2, 15*max_landmark_num);
for k = 1:15*max_landmark_num
    radar_noise(:, :, k) = mvnrnd([0, 0],sensor.R_radar, T);
end
odo_reading = u + odo_noise';
odo_reading = odo_reading * dt;

% generate process noise
process_noise = mvnrnd([0, 0, 0], [1.5e-3 0 0; 0 1.5e-3 0; 0 0 5e-5], T);
measurement_all = cell(1, T); % Radar measurements
reflection_point = cell(1, T);
clutter_point = cell(1, T);

% Generate robot motion data and measurement data
theta_start = [0 0 0]';
theta_vector = zeros(3, T); % robot pose state
state_truth_vector = zeros(3, T);

for t=1:T
    % compute robot pose, according to the dynamic model (control input is
    % provided)
    if t==1
        theta_vector(:, 1) = theta_start;
        x = theta_vector(1,t);
        y = theta_vector(2,t);
        phi = theta_vector(3,t);
    else
        x = theta_vector(1, t-1) + dt*u(1, t-1) * cos(theta_vector(3, t-1) + dt*u(2, t)/2) + process_noise(t, 1);
        y = theta_vector(2, t-1) + dt*u(1, t-1) * sin(theta_vector(3, t-1) + dt*u(2, t)/2)+ process_noise(t, 2);
        phi = headingControl(theta_vector(3, t-1), dt*u(2, t-1), process_noise(t, 3));
        theta_vector(:, t) = [x, y, phi]';
    end
    
    robot.x = x;
    robot.y = y;
    robot.heading = phi;
    % generate measurements according to radar parameters: detection
    % range,detection probability
    landmark_groundtruth = landmarks_cell{t};
    data = detection_model(robot, sensor.R_MAX, landmark_groundtruth, sensor.pd, lambda);
    reflection_point{t} = data.data(1:data.sep_index, :);
    clutter_index = find(data.index>data.sep_index);
    all_clutter(t).data = data.data(clutter_index, :);
    clutter_point{t} = all_clutter(t).data;
    
    if data.counter > 0
        landmark_candidate = data.data;  % in (x,y) coordinate
        state_truth = [x, y, phi]';
        state_truth_vector(:, t) = state_truth;
        
        measurement_all{t} = zeros(size(landmark_candidate, 1), 3);
        
        true_num = sum(data.index<=data.sep_index);
        
        for i=1:true_num
            ob.x = landmark_candidate(i, 1);
            ob.y = landmark_candidate(i, 2);
            [r, az, snr] = radar_measurement1(robot, ob, radar_noise(t,:,i), sensor.P0);
            measurement_all{t}(i, :) = [r, az, snr];
        end
        
        for i=true_num+1:size(landmark_candidate, 1)
            ob.x = landmark_candidate(i, 1);
            ob.y = landmark_candidate(i, 2);
            [r, az, snr] = radar_measurement1(robot, ob, [0, 0], sensor.P0);
            measurement_all{t}(i, :) = [r, az, snr];
        end
    end
    
end
% save data_landmark odo_noise radar_noise measurement_all theta_vector reflection_point clutter_point odo_reading state_truth_vector