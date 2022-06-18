function [xu, pu] = ekf_update(xk, pk, z, j, R)
%%% Input
% xk: predicted state
% pk: predicted cov
% z: radar measurement （r, psi)
% j: landmark index associated to z
% R: radar measurement noise covariance
%%% Output
% xu, pu, updated state mean and covariance

robot.x = xk(1);
robot.y = xk(2);
robot.heading = xk(3);
j1 = 3 + (j-1)*2 + 1;
j2 = 3 + (j-1)*2 + 2;
ob.x = xk(j1);
ob.y = xk(j2);
[hj_r, hi_a] = radar_measurement2(robot, ob, sqrt([0, 0]));
hj_x = [hj_r, hi_a];

H = Jacobian_h(xk, j);

% Note that when the azimuth is close to 180, due to noise pertubation,
% abnormal situation may occur at residual computation
v = z - hj_x; 
% I refine the residual in this way, please email me if you have better
% solutions (shuai.sun@dlmu.edu.cn), thank you.
if z(2)>2*pi/3 && hj_x(2)<-2*pi/3 
    v(2) = z(2) - (2*pi + hj_x(2)); % result negative
end
if z(2)<-2*pi/3 && hj_x(2)>2*pi/3
    v(2) = 2*pi + z(2) - hj_x(2);  % positive
end

S = H*pk*H' + R;
W = pk * H' * inv(S);

xu = xk + W * v';
pu = pk - W*S*W';

xu(3) = angle_correction(xu(3));

end

