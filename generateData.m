function [measurement_all, odo_reading, platform_stateVector, landmarks, reflection_point, clutter_point] = generateData(sensor, range, T, dt, lambda)
%% Generate Radar SLAM simulation data
% Inputs:   landmark_groundtruth_1  - first set of ground truth landmark locations
%           landmark_groundtruth_2  - second set of ground truth landmark locations
%           sensor                  - sensor parameters
%           range                   - range of the scene
%           T                       - number of time steps in the simulation
%           dt                      - time interval between measurements
%           lambda                  - clutter strength
%
% Outputs:  measurement_all         - radar measurements from both landmarks and clutter
%           odo_reading             - odometer measurement readings

landmark_change_time = 60;                          % Time at which vehicle leaves the scene
platform_velocity = 4;                              % Velocity of moving platform

% Define the landmark (vehicle) positions and shape in 2D
[landmark_groundtruth_1, landmark_groundtruth_2] = loadLandmarks;
% Set true landmarks at each time point
landmarks = cell(1, T);                             
for k = 1:T
    if k < landmark_change_time
        landmarks{k} = landmark_groundtruth_1;
    else
        landmarks{k} = landmark_groundtruth_2;
    end
end

% Control Input
u = zeros(2, T); 
u(1, :) = platform_velocity;    % Velocity
u(2, 10:100) = 2*dt;            % Angular velocity

% Odometer reading
% Apply the odometer model defined in Eq.(6)
odo_noise = mvnrnd([0, 0], sensor.U_odometer, T);
odo_reading = u + odo_noise';

% Generate process noise
process_noise = mvnrnd([0, 0, 0], sensor.Q_process, T);

measurement_all = cell(1, T);   % Radar measurements in range & azimuth
reflection_point = cell(1, T);  % Measurments from reflections from landmarks in (x, y)
clutter_point = cell(1, T);     % Measurements from clutter in (x, y)

% Generate platform motion data and measurement data
platform_stateVector = zeros(3, T);
platform.x = zeros(1,T);
platform.y = zeros(1,T);
platform.theta = zeros(1,T);

for k = 1:T
    % Compute the platform pose, according to the dynamic model (control input is provided)
    if k == 1
        platform_stateVector(:, 1) = [0; 0; 0];                    % Set state initial state vector values to zero
        platform.x = platform_stateVector(1,k);                    % Set the platform postion in x
        platform.y = platform_stateVector(2,k);                    % Set the platform position in y
        platform.theta = platform_stateVector(3,k);                % Set the platform heading angle, theta
    else
        % Update the positions in x and y and the heading angle
        platform.x = platform_stateVector(1, k-1) + dt*u(1, k-1) * cos(platform_stateVector(3, k-1) + dt*u(2, k)/2) + process_noise(k, 1);
        platform.y = platform_stateVector(2, k-1) + dt*u(1, k-1) * sin(platform_stateVector(3, k-1) + dt*u(2, k)/2) + process_noise(k, 2);
        platform.theta = platform_stateVector(3, k-1) + dt*u(2, k-1) + process_noise(k, 3);
        % Wrap heading if outside of the range [0, 2*pi]
        platform.theta = wrapTo2Pi(platform.theta);
        platform_stateVector(:, k) = [platform.x, platform.y, platform.theta]';    % Update the state vector for time step k
    end

    % Generate detections according to radar parameters: detection range, detection probability
    data = detectionModel(platform, sensor.max_range, range, landmarks{k}, sensor.prob_detect, lambda);
    reflection_point{k} = data.data(1:data.true_num, :);            % Select the data which comes from true reflections
    clutter_point{k} = data.data(data.true_num+1:data.counter, :);  % Select the data from clutter
    
    if data.counter > 0
        measurement_all{k} = zeros(data.counter, 3);                % Set the size of the measurements at the current time step  
        
        % Generate radar noise
        radar_noise = mvnrnd([0, 0], sensor.R_radar, data.true_num);

        for i = 1:data.true_num
            % Calculate the radar measurements of true landmark detections in relation to the platform
            [r, az, snr] = radarMeasurement(platform, data.data(i,1:2), radar_noise(i,:), sensor.P0);
            measurement_all{k}(i, :) = [r, az, snr];
        end
        
        for i = data.true_num+1:data.counter
            % Calculate the radar measurements of the clutter in relation to the platform
            [r, az, snr] = radarMeasurement(platform, data.data(i,1:2), [0, 0], sensor.P0);
            measurement_all{k}(i, :) = [r, az, snr];
        end
    end
    
end
end