function [C] = Euler2DCM(E)
%Euler2DCM 
%   input: vector of phi theta psai
%   unit: rad
C=[cos(E(2))*cos(E(3)) -cos(E(1))*sin(E(3))+sin(E(1))*sin(E(2))*cos(E(3)) sin(E(1))*sin(E(3))+cos(E(1))*sin(E(2))*cos(E(3));
cos(E(2))*sin(E(3)) cos(E(1))*cos(E(3))+sin(E(1))*sin(E(2))*sin(E(3)) -sin(E(1))*cos(E(3))+cos(E(1))*sin(E(2))*sin(E(3));
-sin(E(2)) sin(E(1))*cos(E(2)) cos(E(1))*cos(E(2));];
end

