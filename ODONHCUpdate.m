function kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, dt)

    param = Param();

    %% measurement innovation
    win_n = winn(navstate);
    wib_b = thisimu(2:4,1) / dt;
    win_b = navstate.Cbn' * win_n;
    wnb_b = wib_b - win_b;
    
    Z = cfg.Cbv*navstate.Cbn'*navstate.vel + cfg.Cbv*skew(wnb_b)*cfg.odolever - odonhc_vel;

    %% measurement equation and noise
    R = diag(power(cfg.odonhc_measnoise, 2));
    H = zeros(3, kf.RANK);
    H(:, 4:6) = cfg.Cbv*navstate.Cbn';
    H(:, 7:9) = -cfg.Cbv*navstate.Cbn'*skew(navstate.vel);
    H(:, 10:12) = -cfg.Cbv*skew(cfg.odolever);
    H(:, 16:18) = -cfg.Cbv*skew(cfg.odolever)*diag(wib_b);
    H(:, 22) = -odonhc_vel;  %%%

    %% judge whether use single odo/nhc
    % only use nhc
    if cfg.usesinglenhc && ~cfg.usesingleodo
        Z = [Z(2); Z(3)];
        R = R(2:3, 2:3);
        H = H(2:3, :);
    end
    % only use odo
    if ~cfg.usesinglenhc && cfg.usesingleodo
        Z = Z(1);
        R = R(1, 1);
        H = H(1, :);
    end

    %% update
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - H * kf.x);
    kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';  
    
end












