function [Rx, Ry, Rz] = rotation_matrices(alpha, beta, gamma)

    % Rotation about X-axis (Roll)
    Rx = [1  0           0;
          0  cos(alpha)  -sin(alpha);
          0  sin(alpha)   cos(alpha)];
    
    % Rotation about Y-axis (Pitch)
    Ry = [cos(beta)  0  sin(beta);
          0          1  0;
         -sin(beta)  0  cos(beta)];
    
    % Rotation about Z-axis (Yaw)
    Rz = [cos(gamma)  -sin(gamma)  0;
          sin(gamma)   cos(gamma)  0;
          0           0           1];
end
