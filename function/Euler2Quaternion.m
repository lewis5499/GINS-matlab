function q = Euler2Quaternion(E)
% input roll,pitch,raw
% uint:deg
q = [cos(E(1)/2)*cos(E(2)/2)*cos(E(3)/2)+sin(E(1)/2)*sin(E(2)/2)*sin(E(3)/2);
     sin(E(1)/2)*cos(E(2)/2)*cos(E(3)/2)-cos(E(1)/2)*sin(E(2)/2)*sin(E(3)/2);
     cos(E(1)/2)*sin(E(2)/2)*cos(E(3)/2)+sin(E(1)/2)*cos(E(2)/2)*sin(E(3)/2);
     cos(E(1)/2)*cos(E(2)/2)*sin(E(3)/2)-sin(E(1)/2)*sin(E(2)/2)*cos(E(3)/2)];