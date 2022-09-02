%% Initialization of landmark management & EKF
id_list = [];                           % List of the IDs of the current landmarks
landmark_ID_No = 0;                     % Total number of landmarks used to generate landmark IDs
unconfirmed_count = 0;                  % Number of unconfirmed landmarks
id_list_count = 0;                      % Number of IDs in the landmark ID list

% Initialize unconfirmed landmarks
unconfirmed_landmarks(T) = struct('centerinfo', [], ...
    'association_counter', [], ...
    'window_length', [], ...
    'detection_points', []);  

% Initialize confirmed landmarks (100 is the assumed maximum landmarks in the scene)
confirmed_landmarks(100) = struct('ID', [], ...
    'createTime', [], ...
    'removeTime', [], ...
    'position', [], ...
    'association', [], ...
    'withinRange', []);

% Initialize state vector
state(T) = struct('x_predict', [], ...
    'cov_predict', [], ...
    'x_update', [], ...
    'cov_update', []); 
% Assume the initial pose of the robot is known
state(1).x_predict = [0.1, 0.1, pi/180]';   % Initial state vector prediction
state(1).cov_predict = [0.15^2 0 0;         % Initial covariance prediction
                        0 0.15^2 0;
                        0 0 (pi/180)^2];
