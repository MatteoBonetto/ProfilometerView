function H = generate_H(t, angle_x, angle_y, angle_z)
        % create a 4*4 homogeneous matrix from a 3*1 vector and a 3*3
        % rotation matrix
        H = eye(4);
        rot = rotz(angle_z) * roty(angle_y) * rotx(angle_x);
        H(1:3, 1:3) = rot;
        H(1:3, 4) = t;
    end