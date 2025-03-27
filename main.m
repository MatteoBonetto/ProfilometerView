clc
clear all;
close all;

% Initialization and initial view
laser_W_H = generate_H([-0 0 0], 20, 0, 0);
camera_laser_H_2640 = generate_H([0 102 6], 0.19 * 180 / pi, 0, 0);
angle_range_2640 = 0.48 * 180 / pi;
Gocator = ProfilometerView(laser_W_H, camera_laser_H_2640, angle_range_2640);
Gocator.import_stl("S49C-MD.stl");
Gocator.view();
% view after centering stl
Gocator.center_stl();
Gocator.view();
% view after moving stl
H_stl = generate_H([0 0 1000], 0, 90, 90);
Gocator.move_stl(H_stl);
Gocator.view();
% view after scanning
Gocator.perform_scan();
Gocator.view();
% view from the camera point of view
Gocator.camera_view();
