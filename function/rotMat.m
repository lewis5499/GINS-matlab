function mat_rotated = rotMat(name, angle, axis)

if axis=="right"
    % NED-R
    if name=="roll"
        mat_rotated=[ 1,       0,            0;
                      0,  cos(angle),  -sin(angle);
                      0,  sin(angle),   cos(angle)];
    elseif name=="pitch"
        mat_rotated=[ cos(angle),   0,   sin(angle);
                          0,        1,       0;
                     -sin(angle),   0,   cos(angle)];
    elseif name=="yaw"
        mat_rotated=[ cos(angle),  -sin(angle),    0;
                      sin(angle),   cos(angle),    0;
                          0,            0,         1];
    end
end

if axis=="left"
    %% NEU-L
    if name=="roll"
        mat_rotated=[ 1,       0,            0;
                      0,   cos(angle),   sin(angle);
                      0,  -sin(angle),   cos(angle)];
    elseif name=="pitch"
        mat_rotated=[ cos(angle),   0,   -sin(angle);
                          0,        1,       0;
                      sin(angle),   0,   cos(angle)];
    elseif name=="yaw"
        mat_rotated=[  cos(angle),   sin(angle),    0;
                      -sin(angle),   cos(angle),    0;
                           0,            0,         1];
    end

end
end

