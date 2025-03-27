classdef ProfilometerView  < handle

    properties
        laser_W_H_
        camera_laser_H_
        camera_W_H_
        vertices_
        faces_
        axis_scale_
        axis_width_
        intersection_points_
        angle_laser_range_ % degree
    end

    methods
        function obj = ProfilometerView(laser_H, camera_laser_H, angle_laser)
            % Input
            obj.laser_W_H_ = laser_H;
            obj.camera_laser_H_ = camera_laser_H;
            obj.camera_W_H_ = obj.laser_W_H_ * obj.camera_laser_H_;
            obj.angle_laser_range_ = angle_laser;
            % Default
            obj.axis_scale_ = 100;
            obj.axis_width_ = 3;
            obj.intersection_points_ = [];
        end

        function [] = view(obj)
            figure;
            % Plot 3D RF of laser
            draw3dReferenceSystems( obj.laser_W_H_ , "laser RF" , [obj.axis_scale_ obj.axis_scale_ obj.axis_scale_], obj.axis_width_);
            draw3dReferenceSystems( obj.camera_W_H_, "camera RF" , [obj.axis_scale_ obj.axis_scale_ obj.axis_scale_], obj.axis_width_);
            axis equal;  % Keep aspect ratio
            xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
            grid on;
            % Plot the STL mesh
            if ~isempty(obj.vertices_) && ~isempty(obj.faces_)
                trisurf(obj.faces_, obj.vertices_(:,1), obj.vertices_(:,2), obj.vertices_(:,3), ...
                'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

                light; lighting phong;  % Add lighting effects
                camlight('headlight');  % Improve shading visibility
            end
            % Plot the 3D points
            if ~isempty(obj.intersection_points_)
                scatter3(obj.intersection_points_(:, 1), obj.intersection_points_(:, 2), obj.intersection_points_(:, 3), 20, 'red', 'filled'); % Size 20, blue color, filled markers
            end
            hold off;
        end

        function import_stl(obj, path)
            % Load STL file
            stlData = stlread(path);
            
            % Extract vertices and faces
            obj.vertices_ = stlData.Points;
            obj.faces_ = stlData.ConnectivityList;
        end

        function center_stl(obj)
            % Compute the centroid of the mesh
            barycenter = obj.compute_vertex_centroid(obj.vertices_);
        
            % Translate vertices to center at the origin
            obj.vertices_ = obj.vertices_ - barycenter;
        end

        function move_stl(obj, H)
            matrix_vertices_with_ones = [obj.vertices_, ones(size(obj.vertices_, 1), 1)];
            matrix_vertices_with_ones_moved = (H * matrix_vertices_with_ones')';
            obj.vertices_ = matrix_vertices_with_ones_moved(:, 1:3);
        end

        function intersectionPoint = raycast(obj, rayOrigin, rayDir) 
            % Output: 1*3 point && Input: 2 vectors 1*3
            % Initialize intersection data
            minDist = inf;
            intersectionPoint = [];
            
            % Loop through each triangle and check for intersection
            for i = 1:size(obj.faces_, 1)
                % Get triangle vertices
                V1 = obj.vertices_(obj.faces_(i, 1), :);
                V2 = obj.vertices_(obj.faces_(i, 2), :);
                V3 = obj.vertices_(obj.faces_(i, 3), :);
                
                % Check ray-triangle intersection
                [hit, point, t] = obj.rayTriangleIntersection(rayOrigin, rayDir, V1, V2, V3);
                
                % If there is an intersection, check if it's the closest
                if hit && t < minDist
                    minDist = t;
                    intersectionPoint = point;
                end
            end
        end

        function perform_scan(obj)
            obj.intersection_points_ = [];
            % Cycle over angles
            laser_origin = obj.laser_W_H_(1:3, 4);
            laser_Z = obj.laser_W_H_(1:3, 3);
            for i = - obj.angle_laser_range_/2 : 0.01 : obj.angle_laser_range_/2
                laser_direction = roty(i) * laser_Z;
                point = obj.raycast(laser_origin', laser_direction');
                obj.intersection_points_ = [obj.intersection_points_; point];
            end
        end

        function camera_view(obj)
            obj.view();
            % Assume H is your homogeneous transformation matrix (4x4)
            cameraPosition = obj.camera_W_H_(1:3, 4);     % Extract translation (position)
            % cameraPosition = cameraPosition - 1000*obj.camera_W_H_(1:3,
            % 3); view 3D RF
            cameraTarget = cameraPosition + obj.camera_W_H_(1:3, 3); % Point along the Z-axis
            upVector = obj.camera_W_H_(1:3, 2);           % Up direction is the Y-axis
            
            % Set the camera view
            camproj('perspective');         % Use perspective projection
            campos(cameraPosition');        % Set camera position
            camtarget(cameraTarget');       % Set target (where it looks)
            camup(upVector');               % Set the up direction
            
            % Set the field of view
            fov = 0.25 * 180 / pi;  % Example FOV in degrees
            camva(fov); % Set camera view angle
        end

    end

    methods (Static)
        function barycenter = compute_vertex_centroid(vertices)
            % Compute mean of all vertex coordinates
            barycenter = mean(vertices, 1); 
        end

        function [hit, P, t] = rayTriangleIntersection(rayOrigin, rayDir, V1, V2, V3)
            % Function to check ray-triangle intersection (Möller–Trumbore algorithm)
            epsilon = 1e-6;  % Small tolerance
            edge1 = V2 - V1;
            edge2 = V3 - V1;
            h = cross(rayDir, edge2);
            a = dot(edge1, h);
            
            if abs(a) < epsilon
                hit = false; P = []; t = inf;
                return;  % Parallel ray
            end
            
            f = 1 / a;
            s = rayOrigin - V1;
            u = f * dot(s, h);
            
            if u < 0 || u > 1
                hit = false; P = []; t = inf;
                return;
            end
            
            q = cross(s, edge1);
            v = f * dot(rayDir, q);
            
            if v < 0 || u + v > 1
                hit = false; P = []; t = inf;
                return;
            end
            
            t = f * dot(edge2, q);
            
            if t > epsilon  % Intersection found
                P = rayOrigin + t * rayDir;
                hit = true;
            else
                hit = false; P = []; t = inf;
            end
        end
    end
end