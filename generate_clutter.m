function y = generate_clutter(lambda, x_range, y_range, robot, R_MAX)
% Shuai SUN 25/07/2021
% generate sence clutter according to the model:
% noise num is Possion distributed
% measurement is uniformly distributed in space
% the clutter is generated in the whole scene, however, 
% returned data picks only the clutter that lies within the robot vision

y.num = poissrnd(lambda);

if y.num > 0
    xc = x_range(1) + (x_range(2)-x_range(1)) * rand(1, y.num);
    yc = y_range(1) + (y_range(2)-y_range(1)) * rand(1, y.num);
    
    clutter = [xc', yc'];
end

y.clutter = [];

for i=1:y.num
    if sqrt((clutter(i,1)-robot.x)^2 + (clutter(i,2)-robot.y)^2) <= R_MAX
        y.clutter = [y.clutter; clutter(i, :)];
    else
        y.num = y.num - 1;
    end
end

end