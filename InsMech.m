function navstate = InsMech(laststate, lastimu, thisimu, cfg)

    param = Param();

    %% Preparation for adapting the code
    interval = thisimu(1, 1) - lastimu(1, 1);
    deltatheta_k_1 = lastimu(2:4, 1);
    Delta_thetak = thisimu(2:4, 1);
    deltav_k_1 = lastimu(5:7, 1);
    Delta_vk = thisimu(5:7, 1);
    e_omega = param.wie;
    r_half = laststate.pos;
    v_half = laststate.vel;
    Rm_half = laststate.Rm;
    Rn_half = laststate.Rn;
    q_b_n_k_1 = laststate.qbn;
    rk_1 = laststate.pos;
    vk_1 = laststate.vel;
    gravity = laststate.gravity;


    %% Inherited code
    % region
    phi_k = Delta_thetak+cross(deltatheta_k_1,Delta_thetak)/12;
    qbb = [cos(0.5*norm(phi_k));sin(0.5*norm(phi_k))/(0.5*norm(phi_k))*0.5*phi_k];%41
    
    omega_ie_n_half = [e_omega*cos(r_half(1)),0,-e_omega*sin(r_half(1))]';
    omega_en_n_half = [v_half(2)/(Rn_half+r_half(3)),-v_half(1)/(Rm_half+r_half(3)),-v_half(2)*tan(r_half(1))/(Rn_half+r_half(3))]';
    zeta_k = (omega_ie_n_half+omega_en_n_half)*interval;%31
    
    qnn = [cos(0.5*norm(zeta_k));-sin(0.5*norm(zeta_k))/(0.5*norm(zeta_k))*0.5*zeta_k];%41
    
    q_b_n_k = QuaternionMultiply(QuaternionMultiply(qnn,q_b_n_k_1),qbb);
    normq = norm(q_b_n_k);  % denominator
    q_b_n_k = q_b_n_k/normq;

    % velocity
    gl_n_half = [0; 0; gravity];

    %i->k timestamp
    deltav_fk_bk_1 = Delta_vk+0.5*cross(Delta_thetak,Delta_vk)+(cross(deltatheta_k_1,Delta_vk)+cross(deltav_k_1,Delta_thetak))/12;
    ksaik_1_k = (omega_en_n_half+omega_ie_n_half)*interval;
    Cb_n_tk_1 = Quaternion2DCM(q_b_n_k_1);
    deltav_fk_n = (eye(3)-0.5*skew(ksaik_1_k))*Cb_n_tk_1*deltav_fk_bk_1;
    
    deltavg_cor_k = (gl_n_half-cross(2*omega_ie_n_half+omega_en_n_half,v_half))*interval;

    vk=vk_1+deltav_fk_n+deltavg_cor_k;

    % pos
    rk(3,1) = rk_1(3)-0.5*(vk(3)+vk_1(3))*interval;% update h
    mean_h = (rk(3)+rk_1(3))/2;
    rk(1,1) = rk_1(1)+0.5*(vk_1(1)+vk(1))/(getRm(rk_1(1))+mean_h)*interval;% update phi
    mean_phi = (rk(1)+rk_1(1))/2;
    rk(2,1) = rk_1(2)+0.5*(vk(2)+vk_1(2))/((getRn(mean_phi)+mean_h)*cos(mean_phi))*interval;% update lambda
    
    % endregion

    %% Update new value
    navstate.time = thisimu(1, 1);
    navstate.pos = rk;
    navstate.vel = vk;
    navstate.qbn = q_b_n_k;
    navstate.Cbn = Quaternion2DCM(navstate.qbn);
    navstate.att = DCM2Euler(navstate.Cbn);
    navstate.gyrbias = laststate.gyrbias;
    navstate.accbias = laststate.accbias;
    navstate.gyrscale = laststate.gyrscale;
    navstate.accscale = laststate.accscale;
    navstate.odoscale = laststate.odoscale;   %%%
    navstate.Rm = getRm(navstate.pos(1));
    navstate.Rn = getRn(navstate.pos(1));
    navstate.gravity = getGravity(navstate.pos(1, 1), navstate.pos(3, 1));

end















