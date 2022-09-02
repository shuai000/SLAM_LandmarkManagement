function landmarks = detectionModel(platform, max_range, range, all_landmarks, prob_detect, lambda)
%% Given all the ground truth landmarks, picks the landmarks that lie within the platform detection range 
% Inputs:   platform        - platform state vector
%           max_range       - maximum detection range of the radar sensor
%           range           - range of the scene 
%           all_landmarks   - locations of all of the landmarks
%           prob_detect     - probability of detecting a landmark
%           lambda          - clutter strength
%
% Output:   landmarks       - detected landmarks in global x-y coordinate

steps = 0.1;                    % Step size for the range grid - determines how close clutter points can be to each other
la = [0.8, 1.5];                % Controls the mean number of detections per m^2 of a landmark and variance on the mean
PossionParam = lambda * (range(1, 2) - range(1, 1)) * (range(2, 2) - range(2, 1));  % Poisson distribution rate parameter
                 
% Generate landmark based detections
N = size(all_landmarks, 1);     % Number of actual landmarks
landmarks.counter = 0;          % Number of detected landmarks
landmarks.data = [];            % Locations of the detected landmarks
for i = 1:N
    cx = all_landmarks(i,1) + all_landmarks(i,3)/2;                 % Center of landmark in x
    cy = all_landmarks(i,2) + all_landmarks(i,4)/2;                 % Center of landmark in y
    if sqrt((platform.x-cx)^2 + (platform.y - cy)^2) <= max_range   % Check if the landmark is within the detection range
        % Calculate the number of potential detections for the current landmark
        detection_num = floor(mvnrnd(la(1) * all_landmarks(i,3) * all_landmarks(i,4), la(2)));
        % Determine the locations of the potential detections
        temp = all_landmarks(i,1:2) + rand(detection_num, 2) .* all_landmarks(i,3:4);
        for k = 1:detection_num
            if prob_detect > rand   % For each potential detection determine if it is actually detected or not
                landmarks.counter = landmarks.counter + 1;          % Increase number of landmarks detected
                landmarks.data = [landmarks.data; temp(k, :)];      % Add location of detection to landmark detection locations
            end
        end
    end
end
landmarks.true_num = landmarks.counter;         % Number of detected landmarks from true landmarks

% Generate false alarms
NumFalseAlarms = poissrnd(PossionParam);        % Calculate the number of false alarms
rangeDist = range(:, 2) - range(:, 1);          % Calculate the size of the range
gridArrayNum(1,1) = rangeDist(1)/(2*steps);     % Divide the range into a grid 
gridArrayNum(2,1) = rangeDist(2)/(2*steps);
gridArrayNum = floor(gridArrayNum);

for i = 1:NumFalseAlarms
    noiseIndex = gridArrayNum.*rand(2,1);                           % For each potential false alarm select a random grid point
    noiseIndex = floor(noiseIndex);
    temp_location = range(:, 1) +  noiseIndex.*2*steps - steps;     % Create false alarm location
    
    % If false alarm is within the dection range of the platform add it to the detected landmarks
    if sqrt((platform.x-temp_location(1))^2 + (platform.y - temp_location(2))^2) <= max_range
        landmarks.counter = landmarks.counter + 1;
        landmarks.data(landmarks.counter, :) = temp_location';
    end
end

end