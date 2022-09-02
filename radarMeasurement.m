function [range, azimuth, snr] = radarMeasurement(platform, landmark, noise, P0)
%% Calculate the range & azimuth of a radar measurement along with the SNR
% Inputs:   platform    - current pose of the platform
%           landmark    - postion of the landmark in the global x-y coordinate
%           noise       - range and azimuth noise
%           P0          - signal power (optional)
% Outputs:  range       - distance between platform and landmark
%           azimuth     - angle between platform and landmark
%                         on the left hand side of the platform: azimuth positive [0, pi]
%                         on the right hand side of the platform: azimuth negative (-pi, 0)
%           snr         - signal to noise ratio (optional)

% Calculate range between platform and landmark
range = sqrt((platform.x - landmark(1))^2 + (platform.y - landmark(2))^2) + noise(1);
% Calculate azimuth between platform and landmark
if platform.x - landmark(1) == 0
    if platform.y > landmark(2)
        azimuth = -pi/2 + noise(2);
    else
        azimuth = pi/2 + noise(2);
    end
else
    azimuth = atan2( (-platform.y + landmark(2)), (-platform.x + landmark(1))) - platform.theta + noise(2);
end
% Wrap azimuth angle if outside of the range [-pi, pi]
azimuth = wrapToPi(azimuth);

% If required calculate SNR
if exist('P0','var')
    snr = P0 - 20 * log(sqrt((platform.x - landmark(1))^2 + (platform.y - landmark(2))^2));
end
end