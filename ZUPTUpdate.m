function kf = ZUPTUpdate(navstate, kf, cfg, thisimu, dt)

    param = Param();

    %% ZUPT
    Z = navstate.vel - [0; 0; 0];

    R = diag(power(cfg.zupt_vmeasnoise, 2));
    H = zeros(3, kf.RANK);
    H(:, 4:6) = eye(3);

    % update
    K = kf.P * H'/ (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - H*kf.x);
    kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';

    %% ZARU(yaw);
    wib_b = thisimu(4) / dt;
    win_n = winn(navstate);
    win_b = (navstate.Cbn)' * win_n;
    Z = wib_b - win_b(3) - 0;

    R = diag(power(cfg.zupt_wmeasnoise, 2));
    H = zeros(1, kf.RANK);
    H(10:12) = [0, 0, 1];
    H(16:18) = [0, 0, wib_b];

    % update
    K = kf.P * H'/ (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - H*kf.x);
    kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';

end