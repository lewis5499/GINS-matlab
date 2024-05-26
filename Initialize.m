function [kf, navstate] = Initialize(cfg)  
    % kalman parameters initialization
    kf.RANK = 22;           %%%
    kf.NOISE_RANK = 19;     %%%
    kf.P = zeros(kf.RANK, kf.RANK);
    kf.Qc = zeros(kf.NOISE_RANK, kf.NOISE_RANK);
    kf.x = zeros(kf.RANK, 1);

    % Qc
    kf.Qc(1:3, 1:3) = power(cfg.accvrw, 2) * eye(3, 3);
    kf.Qc(4:6, 4:6) = power(cfg.gyrarw, 2) * eye(3, 3);
    kf.Qc(7:9, 7:9) = 2 * power(cfg.gyrbiasstd, 2) / cfg.corrtime * eye(3, 3);
    kf.Qc(10:12, 10:12) = 2 * power(cfg.accbiasstd, 2) / cfg.corrtime * eye(3, 3);
    kf.Qc(13:15, 13:15) = 2 * power(cfg.gyrscalestd, 2) / cfg.corrtime * eye(3, 3);
    kf.Qc(16:18, 16:18) = 2 * power(cfg.accscalestd, 2) / cfg.corrtime * eye(3, 3);
    kf.Qc(19, 19) = power(cfg.odoscalestd, 2); % random walk
    % kf.Qc(19, 19) = 2 * power(cfg.odoscalestd, 2) / cfg.corrtime; % Gaussian Markov process
    
    % P0
    kf.P(1:3, 1:3) = diag(power(cfg.initposstd, 2));
    kf.P(4:6, 4:6) = diag(power(cfg.initvelstd, 2));
    kf.P(7:9, 7:9) = diag(power(cfg.initattstd, 2));
    kf.P(10:12, 10:12) = diag(power(cfg.initgyrbiasstd, 2));
    kf.P(13:15, 13:15) = diag(power(cfg.initaccbiasstd, 2));
    kf.P(16:18, 16:18) = diag(power(cfg.initgyrscalestd, 2));
    kf.P(19:21, 19:21) = diag(power(cfg.initaccscalestd, 2));
    kf.P(22, 22) = diag(power(cfg.initodoscalestd, 2)); %%%

    % navigation state initialization
    navstate.time = cfg.starttime;
    navstate.pos = cfg.initpos;
    navstate.vel = cfg.initvel;
    navstate.att = cfg.initatt;
    navstate.Cbn = Euler2DCM(cfg.initatt);
    navstate.qbn = Euler2Quaternion(cfg.initatt);
    navstate.gyrbias = cfg.initgyrbias;
    navstate.accbias = cfg.initaccbias;
    navstate.gyrscale = cfg.initgyrscale;
    navstate.accscale = cfg.initaccscale;
    navstate.odoscale = cfg.initodoscale; %%%
    navstate.Rm = getRm(cfg.initpos(1));
    navstate.Rn = getRn(cfg.initpos(1));
    navstate.gravity = getGravity(cfg.initpos(1), cfg.initpos(3));
    
end