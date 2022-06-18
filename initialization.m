
% Used for management of landmarks
Lk_detected = cell(1, T);    % landmarks that are detected (not registered)
landmark_list_cell = cell(1, 100); % landmarks that are registered, 100 is the maximum landmarks assumed in the scene
counter_landmark = 0; counter_id = 0;

% Initialization of the EKF filter
% Assume the initial pose of the robot is known
struct_state(1).x_predict = [0.1, 0.1, pi/180]';
struct_state(1).cov_predict = [0.15^2 0 0;
    0 0.15^2 0;
    0 0 (pi/180)^2];
