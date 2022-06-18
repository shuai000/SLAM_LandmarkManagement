function landmarks = detection_model(robot, R_MAX, all_landmarks, pd, lambda)
% Given all the ground truth landmarks, picks the landmarks that lie within
% the robot vision (defined in the circle centered on the robot position)
% Returned data is in X, Y format
Range = [-10, 30; -10 30];
PossionParam = lambda * (Range(1, 2) - Range(1, 1)) * (Range(2, 2) - Range(2, 1));
steps = 0.1;

% for landmarks
la = 0.8;

N = size(all_landmarks, 1);

landmarks.counter = 0;
landmarks.data = [];
landmarks.index = [];
% generate landmark based detections
for i=1:N
    cx = all_landmarks(i,1) + all_landmarks(i,3)/2;
    cy = all_landmarks(i,2) + all_landmarks(i,4)/2;
    if sqrt((robot.x-cx)^2 + (robot.y - cy)^2) <= R_MAX
        detection_num = floor(mvnrnd(la * all_landmarks(i,3) * all_landmarks(i,4), 1.5));
        temp = all_landmarks(i,1:2) + rand(detection_num, 2) .* all_landmarks(i,3:4);
        for k=1:detection_num
            if pd > rand
                landmarks.counter = landmarks.counter + 1;
                landmarks.data = [landmarks.data; temp(k, :)];
            end
        end
    end
end
% generate flase alarms
NumNoise = poissrnd(PossionParam);

rangeDist = Range(:, 2) - Range(:, 1);
gridArrayNum(1,1) = rangeDist(1)/(2*steps);
gridArrayNum(2,1) = rangeDist(2)/(2*steps);
gridArrayNum = floor(gridArrayNum);

landmarks.sep_index = landmarks.counter;

for i = 1 : NumNoise
    noiseIndex = gridArrayNum.*rand(2,1);
    noiseIndex = floor(noiseIndex);
    temp_location = Range(:, 1) +  noiseIndex.*2*steps - steps;
    
    if sqrt((robot.x-temp_location(1))^2 + (robot.y - temp_location(2))^2) <= R_MAX
        landmarks.counter = landmarks.counter + 1;
        landmarks.data(landmarks.counter, :) = temp_location';
    end
end

if landmarks.counter > 0
    landmarks.data = landmarks.data(1:landmarks.counter, :);
    landmarks.index = 1:landmarks.counter;
else
    landmarks.data = [0, 0];
end

end