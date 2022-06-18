
MN_M = 5; MN_N = 3;
% for landmark deleting
mn_m = 10; mn_n = 2;

% define the environment sence range 40m*40m
x_range = [-15, 45];
y_range = [-15, 45];

have_temp_head = 0;

lambda = 0.005; % clutter strength
%% --------  Radar parameter definition
% R_noise = [sqrt(0.03), sqrt(pi/3600)]; % range (m) and azimuth (radian), standard deviation applied 0.03m 0.05 degree
R_noise = [0.5, pi/180]; % range (m) and azimuth (radian), standard deviation applied 0.5m, 1 degree
% define odometer random noise
sigma_delta_s = 0.02;  % unit, m
sigma_delta_theta = 0.008 * pi / 180; % unit: radian

sensor.R_MAX = 20;
sensor.pd = .9; 
sensor.R_radar = [R_noise(1)^2 0; 0 R_noise(2)^2];
sensor.R_radar_filter = [R_noise(1)^2 0; 0 R_noise(2)^2];
sensor.R_odometer_filter = [sigma_delta_s^2, 0; 0 sigma_delta_theta^2];
sensor.R_odometer = [sigma_delta_s^2, 0; 0 sigma_delta_theta^2];
sensor.P0 = 0;
%% --------- Define the odometer bias
Lk_startIndex = 4;

Q = [1.5e-3, 0 0; 0 1.5e-3 0; 0 0 5e-5];

thresholds.alpha = 500;
thresholds.beta = 3;
thresholds.Ns = 3;
thresholds.Na = 5;
thresholds.cluster_ridus = 2.5;
thresholds.point_threshold = 6;
thresholds.association_ridus = 5;
thresholds.sifting_ridus = 3;
thresholds.min_cluster_points = 2;

dt = 0.16;  % the time interval for measurements
T = 120;