
function [range, azimuth] = radar_measurement2(robot, landmark, R)
% robot and landmark positions are given in the x-y coordinate (Global)
% R vecor of length 2, containing the generated range and azimuth noise

range = sqrt((robot.x - landmark.x)^2 + (robot.y - landmark.y)^2) + R(1);

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