function kf = InsPropagate(navstate, thisimu, dt, kf, corrtime, cfg)

    param = Param();
    
    %% Preparation for adapting the code
    pos = navstate.pos;
    vel = navstate.vel;
    Cbn = navstate.Cbn;
    Rm = navstate.Rm;
    Rn = navstate.Rn;
    gravity = navstate.gravity;

    e_omega = param.wie;
    omega_ib = thisimu(2:4, 1) / dt;
    acce_ib = thisimu(5:7) / dt;

    wie_n = [e_omega*cos(pos(1)); 0; -e_omega*sin(pos(1))];
    wen_n = [vel(2)/(Rn+pos(3)); -vel(1)/(Rm+pos(3)); -vel(2)*tan(pos(1))/(Rn+pos(3))];
    win_n = wie_n + wen_n;

    F = zeros(kf.RANK, kf.RANK);
    PHI = eye(kf.RANK, kf.RANK);
    G = zeros(kf.RANK, kf.NOISE_RANK);

    %% Inherited code
    % region
    fvr = zeros(3, 3);
    fvv = zeros(3, 3);
    fphir = zeros(3, 3);
    fphiv = zeros(3, 3);

    frr = [-vel(3)/(Rm+pos(3)),0,vel(1)/(Rm+pos(3));
        vel(2)*tan(pos(1))/(Rn+pos(3)),-(vel(3)+vel(1)*tan(pos(1)))/(Rn+pos(3)),vel(2)/(Rn+pos(3));
        0,0,0];

    fvr(1,1) = -2*vel(2)*e_omega*cos(pos(1))/(Rm+pos(3))-vel(2)^2/((Rn+pos(3))*(Rm+pos(3))*(cos(pos(1)))^2);
    fvr(1,3) = vel(1)*vel(3)/(Rm+pos(3))^2-vel(2)^2*tan(pos(1))/(Rn+pos(3))^2;
    fvr(2,1) = 2*e_omega*(vel(1)*cos(pos(1))-vel(3)*sin(pos(1)))/(Rm+pos(3))+vel(1)*vel(2)/((Rn+pos(3))*(Rm+pos(3))*(cos(pos(1)))^2);
    fvr(2,3) = vel(2)*vel(3)/(Rn+pos(3))^2+vel(1)*vel(2)*tan(pos(1))/(Rn+pos(3))^2;
    fvr(3,1) = 2*vel(2)*e_omega*sin(pos(1))/(Rm+pos(3));
    fvr(3,3) = -vel(2)^2/(Rn+pos(3))^2-vel(1)^2/(Rm+pos(3))^2+2*gravity/(sqrt(Rm*Rn)+pos(3));
    
    fvv(1,1) = vel(3)/(Rm+pos(3));
    fvv(1,2) = -2*e_omega*sin(pos(1))-2*vel(2)*tan(pos(1))/(Rn+pos(3));
    fvv(1,3) = vel(1)/(Rm+pos(3));
    fvv(2,1) = 2*e_omega*sin(pos(1))+vel(2)*tan(pos(1))/(Rn+pos(3));
    fvv(2,2) = (vel(3)+vel(1)*tan(pos(1)))/(Rn+pos(3));
    fvv(2,3) = 2*e_omega*cos(pos(1))+vel(2)/(Rn+pos(3));
    fvv(3,1) = -2*vel(1)/(Rm+pos(3));
    fvv(3,2) = -2*e_omega*cos(pos(1))-2*vel(2)/(Rn+pos(3));
    
    fphir(1,1) = -e_omega*sin(pos(1))/(Rm+pos(3));
    fphir(1,3) = vel(2)/(Rn+pos(3))^2;
    fphir(2,3) = -vel(1)/(Rm+pos(3))^2;
    fphir(3,1) = -e_omega*cos(pos(1))/(Rm+pos(3))-vel(2)/((Rn+pos(3))*(Rm+pos(3))*(cos(pos(1)))^2);
    fphir(3,3) = -vel(2)*tan(pos(1))/(Rn+pos(3))^2;
    
    fphiv(1,2) = 1/(Rn+pos(3));
    fphiv(2,1) = -1/(Rm+pos(3));
    fphiv(3,2) = -tan(pos(1))/(Rn+pos(3));
    
    F(1:3,1:3) = frr;
    F(4:6,1:3) = fvr;
    F(4:6,4:6) = fvv;
    F(7:9,1:3) = fphir;
    F(7:9,4:6) = fphiv;
    F(1:3,4:6) = eye(3);
    
    F(4:6,7:9) = skew(Cbn*acce_ib);
    F(7:9,7:9) = -skew(win_n);
    F(7:9,10:12) = -Cbn;
    F(4:6,13:15) = Cbn;
    F(7:9,16:18) = -Cbn*diag(omega_ib);
    F(4:6,19:21) = Cbn*diag(acce_ib);

    F(10:12,10:12) = -1/corrtime*eye(3); 
    F(13:15,13:15) = -1/corrtime*eye(3); 
    F(16:18,16:18) = -1/corrtime*eye(3); 
    F(19:21,19:21) = -1/corrtime*eye(3);

    F(22, 22) = 0;  % random walk
    % F(22, 22) = -1/corrtime; % Gaussian Markov process

    % endregion

    %% propagate covariance
    PHI = PHI + F * dt;
    G(4:6, 1:3) = Cbn;
    G(7:9, 4:6) = Cbn;
    G(10:12, 7:9) = eye(3);
    G(13:15, 10:12) = eye(3);
    G(16:18, 13:15) = eye(3);
    G(19:21, 16:18) = eye(3);
    G(22, 19) = 1.0;            %%%

    Q = G * kf.Qc * G' * dt;

    kf.P = PHI * kf.P * PHI' + Q;

end