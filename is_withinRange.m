function y = is_withinRange(robot_p, landmark_p, r)

dist = sqrt((robot_p(1)-landmark_p(1))^2 + (robot_p(2) - landmark_p(2))^2);

if dist <= r
    y = 1;
else
    y = 0;
end

end

