%%% Sensor model applied to obtain range, azimuth and SNR
%%% Sensor heading is considered
%%% Definition: azimuth is within (-pi, pi]
%%% SNR is related to the sensor to landmark distance
%%%              .....>
%%% Please refer to the angle model defined in a paper draft from Shuai SUN
%%% 08/09/2021 
%%% Modified 06/12/2021, adding a SNR

function [range, azimuth, snr] = radar_measurement1(robot, landmark, R, P0)
% robot and landmark positions are given in the x-y coordinate (Global)
% R vecor of length 2, containing the generated range and azimuth noise
% P0 is used for SNR purpose

range = sqrt((robot.x - landmark.x)^2 + (robot.y - landmark.y)^2) + R(1);
snr = P0 - 20 * log(sqrt((robot.x - landmark.x)^2 + (robot.y - landmark.y)^2));

% on the left hand side of the robot: azimuth positive [0, pi]
% on the right hand side of the robot, azimuth negative (-pi, 0)
if robot.x - landmark.x == 0
    if robot.y > landmark.y
        azimuth = -pi/2 + R(2);
    else
        azimuth = pi/2 + R(2);
    end
else
    azimuth = atan2( (-robot.y + landmark.y), (-robot.x + landmark.x)) - robot.heading + R(2);
end

if azimuth > pi
    azimuth = azimuth - 2*pi;
end

if azimuth <= -pi
    azimuth = azimuth + 2*pi;
end

end