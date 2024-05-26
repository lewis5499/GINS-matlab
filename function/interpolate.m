% -------------------------------------------------------------------------
% Author: Liqiang Wang, 
% GNSS Research Center, Wuhan University, China.;
% Contact: wlq@whu.edu.cn;
% Date: 2023.3.9;
% -------------------------------------------------------------------------

function [firstimu, secondimu] = interpolate(lastimu, thisimu, intertime)
    firstimu = zeros(7, 1);
    secondimu = zeros(7, 1);
    if (intertime >= lastimu(1, 1) && intertime <= thisimu(1, 1))
        lambda = (intertime - lastimu(1, 1)) / (thisimu(1, 1) - lastimu(1, 1));
        firstimu(1, 1) = intertime;
        firstimu(2:7, 1) = thisimu(2:7, 1) * lambda;
        secondimu(1, 1) = thisimu(1, 1);
        secondimu(2:7, 1) = thisimu(2:7, 1) * (1 - lambda);
    end
end