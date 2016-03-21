%% Clean Matlab
clear all
close all
clc

%% Plot in 3D several trajectories generated

% Directory information
dir_data.root = '/home/cmastalli/ROS_myThesis/motion_lib';
dir_data.package_name = 'dynamic_movement_primitives';
dir_data.data_directory_name = 'test/data_test';

% Trajectory demostrated
dir_data.doc_name = 'truck_loading_action.txt';
figure('Name', 'Trajectories generated')
plotTrajectory(dir_data, 'k', '3D');

% Right-Left Truck Position
% Trajectory generated 1
dir_data.doc_name = '1rb_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '3D');

% Trajectory generated 2
dir_data.doc_name = '2rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '3D');

% Trajectory generated 3
dir_data.doc_name = '3rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '3D');

% Trajectory generated 4
dir_data.doc_name = '4rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '3D');

% Trajectory generated 5
dir_data.doc_name = '5rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'b', '3D');

% Trajectory generated 6
dir_data.doc_name = '6rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'b', '3D');

% Trajectory generated 7
dir_data.doc_name = '7rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'b', '3D');

% Trajectory generated 8
dir_data.doc_name = '8rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'b', '3D');


% Configure the plot3D
set(gca,'DataAspectRatio',[1 1 1])
set(gca,'CameraUpVector',[0 1 0])
set(gca,'CameraPosition',[20.3288 14.6317 -15.6895])
set(gca,'Projection','perspective')
title('Trajectories generated and demostrated')
xlabel('x')
ylabel('y')
zlabel('z')
grid

%% Plot 2D trajectories generated
% Trajectory demostrated
dir_data.doc_name = 'truck_loading_action.txt';
figure('Name', 'Trajectories generated 2D')
plotTrajectory(dir_data, 'k', '2D');

% Trajectory generated 1
dir_data.doc_name = '1rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '2D');

% Trajectory generated 2
dir_data.doc_name = '2rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '2D');

% Trajectory generated 3
dir_data.doc_name = '3rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '2D');

% Trajectory generated 4
dir_data.doc_name = '4rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, 'g', '2D');

% Trajectory generated 5
dir_data.doc_name = '5rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, '--b', '2D');

% Trajectory generated 6
dir_data.doc_name = '6rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, '--b', '2D');

% Trajectory generated 7
dir_data.doc_name = '7rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, '--b', '2D');

% Trajectory generated 8
dir_data.doc_name = '8rf_truck_loading_dmp_traj_learned.txt';
plotTrajectory(dir_data, '--b', '2D');