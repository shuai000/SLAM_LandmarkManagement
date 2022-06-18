function d = log_distance(zk, xk, Pk, Lk, R, j)
%%% Compute the distance between a given detection and a registered landmark
% zk: 1*2 vector
% xk: mk*1 column vector (length changes with time)
% Lk: n_lk * 2 landmark matrix
% R: 2*2 Radar measurement noise
% j, the jth landmark in xk

robot.x = xk(1); robot.y = xk(2); robot.heading = xk(3);

ob.x = Lk(1); ob.y = Lk(2);

[hj_r, hi_a] = radar_measurement2(robot, ob, sqrt([0, 0]));
hj_x = [hj_r, hi_a];

ek = (zk - hj_x);
% 13/12/2021 because of the azimuth is close to 180, noise pertubed to -, leading to a much larger resisude than it should be
if zk(2)>2*pi/3 && hj_x(2)<-2*pi/3 
    ek(2) = zk(2) - (2*pi + hj_x(2)); % result negative
end
if zk(2)<-2*pi/3 && hj_x(2)>2*pi/3
    ek(2) = 2*pi + zk(2) - hj_x(2);  % positive
end
H = Jacobian_h(xk, j);
Re = R + H * Pk * H';
g = 1 / (sqrt( (2*pi)^2 * det(Re)));
ex = -0.5 * ek * inv(Re) * ek';
d = -log(g * exp(ex));

end