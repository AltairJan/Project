function theta = angle_points_3d(A, B, C)
    % Function to calculate the angle ?ABC in the range [0, 2?].
    % A, B, C are 3D points specified as 1x3 vectors.
    
    % Vectors AB and BC
    AB = A - B;
    BC = C - B;
    
    % Normal vector (cross product)
    normal = cross(AB, BC);
    
    % Cosine of the angle using dot product
    cos_theta = dot(AB, BC) / (norm(AB) * norm(BC));
    % Ensure numerical stability for acos
    cos_theta = max(min(cos_theta, 1), -1);
    
    % Sine of the angle using the norm of the cross product
    sin_theta = norm(normal) / (norm(AB) * norm(BC));
    
    % Calculate the angle using atan2
    theta = atan2(sin_theta, cos_theta);
    
    % Convert angle to the range [0, 2?]
    if theta < 0
        theta = theta + 2 * pi;
    end
end
