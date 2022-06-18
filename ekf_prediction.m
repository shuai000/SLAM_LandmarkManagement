function [X, P] = ekf_prediction(xbk, Pbk, uk, U, Q0)
% state prediction based on EKF filter and motion model
% input:
% uk = [deltaVk; deltaThetak] 2*1 specify odometer reading
% U: Covariance matrix of odeometer reading 
% Q0: Covariance matrix for robot process noise
% output: mean X and covariance P of the predicted state

X = xbk;
N = length(xbk);
uT_s = uk(1);
uT_theta = uk(2);
psi = xbk(3);

Q = zeros(N, N);
Q(1:3, 1:3) = Q0;
% Apply the robot motion model below to predict robot pose
X(1) = xbk(1) + uT_s * cos(psi);
X(2) = xbk(2) + uT_s * sin(psi);
X(3) = angle_correction(xbk(3) + uT_theta);

Fx = zeros(N, N);
Fx(1,1) = 1; Fx(1,3) = -uT_s *sin(psi);
Fx(2,2) = 1; Fx(2,3) = uT_s*cos(psi); 
Fx(3,3) = 1;
for k=4:N
    Fx(k, k) = 1;
end

% 5*2 Jacobin w.r.t the control input
Fu = zeros(N, 2);
Fu(1,1) = cos(psi); Fu(2,1) = sin(psi); Fu(3,2) = 1;

P = Fx * Pbk * Fx' + Fu * U * Fu' + Q;

end