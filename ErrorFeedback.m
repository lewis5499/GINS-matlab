function [kf, navstate] = ErrorFeedback(kf, navstate, cfg)

    % position and velocity
    DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
    navstate.pos = navstate.pos - DR \ kf.x(1:3, 1);
    navstate.vel = navstate.vel - kf.x(4:6, 1);

    % attitude
    qpn = Rotvec2Quat(kf.x(7:9, 1));
    navstate.qbn = QuaternionMultiply(qpn, navstate.qbn);
    navstate.Cbn = Quaternion2DCM(navstate.qbn);
    navstate.qtt = DCM2Euler(navstate.Cbn);

    % imu error
    navstate.gyrbias = navstate.gyrbias + kf.x(10:12, 1);
    navstate.accbias = navstate.accbias + kf.x(13:15, 1);
    navstate.gyrscale = navstate.gyrscale + kf.x(16:18, 1);
    navstate.accscale = navstate.accscale + kf.x(19:21, 1);

    % odo scale factor
    navstate.odoscale = navstate.odoscale + kf.x(22, 1);   %%%

    % update some parameters
    navstate.Rm = getRm(navstate.pos(1));
    navstate.Rn = getRn(navstate.pos(1));
    navstate.gravity = getGravity(navstate.pos(1), navstate.pos(3));

    % reset state vector
    kf.x = zeros(kf.RANK, 1);
end