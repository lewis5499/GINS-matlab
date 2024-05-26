function [matrix33] = skew(vector31)
if size(vector31,2) > size(vector31,1)
    vector31 = vector31';
end
%UNTITLED3 skew symmetric
matrix33 = zeros(3,3);
matrix33(3,2)= vector31(1,1);matrix33(2,3)= -vector31(1,1);
matrix33(1,3)= vector31(2,1);matrix33(3,1)= -vector31(2,1);
matrix33(2,1)= vector31(3,1);matrix33(1,2)= -vector31(3,1);
end

