function [landmarks, count] = deleteUnconfirmedLandmarks(landmarks, deleting_index, count, M, N)
%% Remove unconfirmed landmarks that have now been confirmed and according to MN logic
% Inputs:   landmarks       - current unconfirmed landmarks
%           deleting_index  - array indicating unconfirmed landmarks to be removed
%           count           - current number of unconfirmed landmarks
%           M               - length of MN logic window
%           N               - number of detections required within the MN logic window
%
% Outputs:  landmarks       - updated unconfirmed landmarks
%           count           - updated number of unconfirmed landmarks

% Increase the window length for unconfirmed landmarks
landmarks.window_length = landmarks.window_length + 1;
            
% Check any landmarks which need to be deleted according to MN logic
for i = 1:length(landmarks.association_counter)
    if landmarks.window_length(i) >=M || landmarks.association_counter(i) + M - landmarks.window_length(i) < N
        % If the length of the window for the unconfirmed landmark is greater than the MN logic window length it can be removed
        % If the remaining number of time points in the window is not sufficint to reach the MN logic number of detections the landmark can be removed
        deleting_index(i) = 1;
    end
end
deleting_index = find(deleting_index==1);

% Delete each of the fields for the landmarks that need to be removed
landmarks.association_counter(deleting_index) = [];
landmarks.centerinfo(deleting_index, :)  = [];
landmarks.window_length(deleting_index) = [];
landmarks.detection_points(deleting_index) = [];
count = count - length(deleting_index);

end