function [landmark_groundtruth_1, landmark_groundtruth_2] = loadLandmarks()
%% Define locations of vehicles (landmarks)
% Outputs:  landmark_groundtruth_1  - first set of landmarks
%           landmark_groundtruth_2  - second set of landmarks after one
%                                     vehicle has been removed
% Each set contains 5 groups of vehicles
% Each group contains several vehicles
% Each vehicle is represented by a rectangle

% Vehicle dimensions
vehicle_width = 2;
small_vehicle_length = 4;
large_vehicle_length = 5;

% Location of the first vehicle in the first group
group1_h = 30;
group1_v = 0;
% Location of the first vehicle in the second group
group2_h = 5;
group2_v = 5;
% Location of the first vehicle in the third group
group3_h = -14;
group3_v = 4;
% Location of the first vehicle in the fourth group
group4_h = 2;
group4_v = -8;
% Location of the first vehicle in the fifth group
group5_h = 4;
group5_v = 33;

% First group with 2 small vehicles positioned vertically in relation to each other and parked horizontally
group1 = [group1_h, group1_v, small_vehicle_length, vehicle_width;
    group1_h, group1_v+15, small_vehicle_length, vehicle_width];

% Second group with 3 small vehicles positioned vertically in relation to each other and parked horizontally
group2 = [group2_h, group2_v, small_vehicle_length, vehicle_width;
    group2_h, group2_v+5, small_vehicle_length, vehicle_width;
    group2_h, group2_v+10, small_vehicle_length, vehicle_width];

% Third group with 3 small vehicles positioned vertically in relation to each other and parked vertically
group3 = [group3_h, group3_v, vehicle_width, small_vehicle_length;
    group3_h, group3_v+8, vehicle_width, small_vehicle_length;
    group3_h, group3_v+14, vehicle_width, small_vehicle_length];

% Fourth group with 2 large vehicles positioned horizontally in relation to each other and parked horizontally
group4 = [group4_h, group4_v, large_vehicle_length, vehicle_width;
    group4_h+9, group4_v, large_vehicle_length, vehicle_width];

% Fifth group with 2 large vehicles positioned horizontally in relation to each other and parked horizontally
group5 = [group5_h, group5_v, large_vehicle_length, vehicle_width;
    group5_h+10, group5_v, large_vehicle_length, vehicle_width];

% Create the ground truth for the landmarks
landmark_groundtruth_1 = [group1; group2; group3; group4; group5];
% Remove 1 vehicle from group 3
group3 = group3(2:3,:);
% Create a second ground truth without this vehicle
landmark_groundtruth_2 = [group1; group2; group3; group4; group5];
