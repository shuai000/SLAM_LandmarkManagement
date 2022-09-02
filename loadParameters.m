%% Set the algorithm parameters, see Table I in the paper for more details

range = [-15, 45;                                           % Define the environment scene range 60m*60m
         -15, 45];
dt = 0.16;                                                  % Time interval for measurements
T = 120;                                                    % Number of time steps
lambda = 0.005;                                             % Clutter strength
platform_stateLength = 3;                                   % Length of the vector representing the platform state
num_landmarkParam = 2;                                      % Number of parameters representing each landmark

%% Sensor parameter definitions
sensor.max_range = 20;                                      % Maximum radar detection range (m)
sensor.prob_detect = 0.9;                                   % Probability of detecting a landmark
sensor.R_radar = [0.5^2 0; 0 (pi/180)^2];                   % Radar noise covariance (range variance m^2, azimuth variance radians^2)
sensor.U_odometer = [0.02^2, 0; 0 (0.008 * pi/180)^2];      % Odometer noise covariance (velocity (m/s)^2, yaw rate variance (radians/s)^2)
sensor.Q_process = [1.5e-3, 0 0; 0 1.5e-3 0; 0 0 5e-5];     % Process noise covariance
sensor.P0 = 0;                                              % Signal power

%% Filter parameter definitions
filter.R_radar = sensor.R_radar;                            % Assume radar sensor noise is known
filter.U_odometer = sensor.U_odometer;                      % Assume odometer noise is known
filter.Q_process = sensor.Q_process;                        % Assume process noise is known

%% Thresholds
thresholds.sifting_radius = 3;                              % Threshold for radius of initial sift before data association
thresholds.cluster_radius = 2.5;                            % Threshold for radius of neighborhood for clustering
thresholds.merge_radius = 3;                                % Threshold for radius of merging
thresholds.merge_dist = 0.8;                                % Threshold one axis must be less than for merging
thresholds.alpha = 500;                                     % Threshold for clusters to be far enough away from existing landmarks
thresholds.beta = 3;                                        % Threshold for the log likelihood distance in the sifting
thresholds.Ns = 3;                                          % The number of landmarks to use in sifting 
thresholds.Na = 5;                                          % The number of landmarks to use in association
thresholds.min_cluster_points = 2;                          % Minimum number of measurements to be considered a cluster
thresholds.point_threshold = 6;                             % Number of measurements required to be confirmed automatically as a landmark
thresholds.association_radius = 5;

%% MN logic
MN_logic.M_association = 5;                                 % MN logic M for landmark association
MN_logic.N_association = 3;                                 % MN logic N for landmark association
MN_logic.M_deleting = 10;                                   % MN logic M for landmark deletion
MN_logic.N_deleting = 2;                                    % MN logic N for landmark deletion