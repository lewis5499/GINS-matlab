function g = getGravity(B,H)
grav = [9.7803267715,0.0052790414,0.0000232718,-0.000003087691089,0.000000004397731,0.000000000000721];

sinB=sin(B);
sinB2=sinB^2;
sinB4=sinB^4;
g = grav(1)*(1 + grav(2)*sinB2 + grav(3)*sinB4) + (grav(4) + grav(5)*sinB2)*H + grav(6)*H*H;