function [xu, Pu] = ekfUpdate(xk, Pk, z, H, R, h_x)
%% Use associated radar measurements to perform the EKF state update
% Inputs:   xk                      - predicted state
%           pk                      - predicted cov
%           z                       - radar measurement（range, azimuth)
%           H                       - Jacobian of the measurement model
%           R                       - radar measurement noise covariance
%           h_x                     - location of the landmark in range and azimuth
%
% Outputs:  xu                      - updated state mean
%           Pu                      - updated state covariance

v = z - h_x;                                                    % Calculate residual
% Note that when the azimuth is close to pi, due to noise pertubation, abnormal situations may occur at residual computation
% I refined the residual in this way, please email me if you have a better solution (shuai.sun@dlmu.edu.cn), thank you.
if z(2)>2*pi/3 && h_x(2)<-2*pi/3 
    % Prevent azimuth residual from being large if measurement is close to pi and landmark is close to -pi  
    v(2) = z(2) - (2*pi + h_x(2));                              % Add 2pi to landmark giving a small negative result
elseif z(2)<-2*pi/3 && h_x(2)>2*pi/3
    % Prevent azimuth residual from being large if measurement is close to -pi and landmark is close to pi  
    v(2) = 2*pi + z(2) - h_x(2);                                % Add 2pi to measurement giving a small negative result
end

S = H*Pk*H' + R;                                                % Calculate residual covariance
W = Pk * (H' / S);                                              % Calculate Kalman gain

xu = xk + W * v';                                               % Update the state vector
xu(3) = wrapTo2Pi(xu(3));                                       % Wrap angle if outside of range [0 2pi]
Pu = Pk - W*S*W';                                               % Update the state covariance 

end

