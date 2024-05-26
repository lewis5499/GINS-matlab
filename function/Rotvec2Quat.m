% -------------------------------------------------------------------------
% Author: Liqiang Wang, 
% GNSS Research Center, Wuhan University, China.;
% Contact: wlq@whu.edu.cn;
% Date: 2023.3.5;
% -------------------------------------------------------------------------

function quat = Rotvec2Quat(vec)
    angle = norm(vec);

    if angle < 0.000005
        index = 0.5;
    else
        index = sin(0.5 * angle) / angle;
    end
    
    quat = zeros(4, 1);
    quat(1) = cos(0.5 * angle);
    quat(2) = index * vec(1);
    quat(3) = index * vec(2);
    quat(4) = index * vec(3);
end