function in_range = withinRange(platform, landmark, max_range)
%% Check whether a landmark is within detection range of the platform
% Inputs:   platform    - platform location
%           landmark    - landmark_location
%           max_range   - maximum detection range of the radar sensor
%
% Output:   in_range    - indicator for whether the landmark is in range or not

% Calculate the Euclidean distance between the platform and the landmark
dist = sqrt((platform(1)-landmark(1))^2 + (platform(2) - landmark(2))^2);
% Check whether the distance is less than the detection range or not
if dist <= max_range
    in_range = 1;
else
    in_range = 0;
end

end

