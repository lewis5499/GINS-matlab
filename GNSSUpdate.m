function kf = GNSSUpdate(navstate, gnssdata, kf, antlever, usegnssvel, thisimu, dt, cfg)

    param = Param();

    %% GNSS position update
    % abandon gnss vel outlier 
    gnssposstd = gnssdata(5:7, 1);
    if gnssposstd(1, 1) > 5 || gnssposstd(2, 1) > 5 || gnssposstd(3, 1) > 5
        disp(['WARNING: Abandon gnss position measurement at: ', num2str(gnssdata(1, 1))]);
    else
        % measurement innovation
        DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
        Z = DR*(navstate.pos - gnssdata(2:4, 1)) + navstate.Cbn*antlever; % DR: BLH -> NED
        
        % measurement matrix and noise matrix
        H = zeros(3, kf.RANK);
        H(1:3, 1:3) = eye(3);
        H(1:3, 7:9) = skew(navstate.Cbn * antlever);

        R = diag(power(gnssdata(5:7, 1), 2)); % m m m

        % judge bad gnss measurements
        if cfg.useodonhc || cfg.useZUPT
            if gnssposstd(1, 1) > 0.9
                disp(['WARNING: Abandon gnss NORTH measurement at: ', num2str(gnssdata(1, 1))]);
                H(1,:) = 0;
                Z(1,:) = 0;
            end
            if gnssposstd(2, 1) > 0.9
                disp(['WARNING: Abandon gnss EAST measurement at: ', num2str(gnssdata(1, 1))]);
                H(2,:) = 0;
                Z(2,:) = 0;
            end
            if gnssposstd(3, 1) > 1.8
                disp(['WARNING: Abandon gnss DOWN measurement at: ', num2str(gnssdata(1, 1))]);
                H(3,:) = 0;
                Z(3,:) = 0;
            end
        end

        % update covariance and state vector
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K * (Z - H * kf.x);
        kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';
    end

    %% GNSS velocity update
    if usegnssvel
        % abandon gnss vel outlier 
        gnssvelstd = gnssdata(11:13, 1);
        if gnssvelstd(1, 1) > 0.5 || gnssvelstd(2, 1) > 0.5 || gnssvelstd(3, 1) > 0.5
            disp(['WARNING: Abandon gnss velocity measurement at: ', num2str(gnssdata(1, 1))]);
        else
        % measurement innovation
        win_n = winn(navstate);
        wib_b = thisimu(2:4,1) / dt;
        Z = navstate.vel - gnssdata(5:7, 1) - skew(win_n) * navstate.Cbn * antlever - navstate.Cbn * skew(antlever) * wib_b;

        % measurement matrix and noise matrix
        Hv3 = -skew(win_n) * skew(navstate.Cbn * antlever) - skew(navstate.Cbn * skew(antlever) * wib_b);
        Hv6 = -navstate.Cbn * skew(antlever) * diag(wib_b);
        H = zeros(3, kf.RANK);
        H(:, 4:6) = eye(3);
        H(:, 7:9) = Hv3;
        H(:, 10:12) = -skew(navstate.Cbn * antlever);
        H(:, 16:18) = Hv6;

        R = diag(power(gnssdata(11:13, 1), 2));

        % judge bad gnss measurements
        if cfg.useodonhc || cfg.useZUPT
            if gnssvelstd(1, 1) > 0.1
                disp(['WARNING: Abandon gnss vn measurement at(ODO/NHC aided): ', num2str(gnssdata(1, 1))]);
                H(1,:) = 0;
                Z(1,:) = 0;
            end
            if gnssvelstd(2, 1) > 0.1
                disp(['WARNING: Abandon gnss ve measurement at(ODO/NHC aided): ', num2str(gnssdata(1, 1))]);
                H(2,:) = 0;
                Z(2,:) = 0;
            end
            if gnssvelstd(3, 1) > 0.3
                disp(['WARNING: Abandon gnss vd measurement at(ODO/NHC aided): ', num2str(gnssdata(1, 1))]);
                H(3,:) = 0;
                Z(3,:) = 0;
            end
        end
        % update covariance and state vector
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K * (Z - H * kf.x);
        kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';  
        end
    end
end