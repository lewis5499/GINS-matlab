% -------------------------------------------------------------------------
% Program: GNSS/INS/ODO/NHC integrated navigation
% Note: Based on the platform of GNSS Research Center, WHU, China.
%                       [PROGRAM MAIN SCRIPT]
% Date: 01/24/2024, Wednesday.
% Author: Liu Hengzhen
% Contact: lewis5499@whu.edu.cn
% -------------------------------------------------------------------------

%% define parameters and load process config
param = Param();
cfg = ProcessConfig();
cfg.Cbv = Euler2DCM(cfg.installangle);

%% load data
% imudata
imufid = fopen(cfg.imufilepath);
imudata = fread(imufid, [7, inf], 'double');
fclose(imufid);
imudata = imudata';
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% gnss data
gnssdata = load(cfg.gnssfilepath);
gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;
if (size(gnssdata, 2) < 13)
    cfg.usegnssvel = false;
end
gnssstarttime = gnssdata(1, 1);
gnssendtime = gnssdata(end, 1);

% odo data
if cfg.useodonhc
    odofid = fopen(cfg.odofilepath);
    ododata = fread(odofid, [2, inf], 'double');
    fclose(odofid);
    ododata = ododata';
end

%% save result
navpath = [cfg.outputfolder, '/NavResult'];
if cfg.usegnssvel
    navpath = [navpath, '_GNSSVEL'];
end
if cfg.useodonhc
    navpath = [navpath, '_ODONHC'];
end
navpath = [navpath, '.nav'];
navfp = fopen(navpath, 'wt');

imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
imuerrfp = fopen(imuerrpath, 'wt');

stdpath = [cfg.outputfolder, '/NavSTD.txt'];
stdfp = fopen(stdpath, 'wt');

%% get process time
% 'start time' and 'end time'
% Capture the intersection interval for all time instances,
% and store it in the configuration struct (cfg).
if imustarttime > gnssstarttime
    starttime = imustarttime;
else
    starttime = gnssstarttime;
end
if imuendtime > gnssendtime
    endtime = gnssendtime;
else
    endtime = imuendtime;
end
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
cfg.endtime = endtime;

if cfg.useodonhc
    odoupdatetime = ceil(cfg.starttime) + 0.5; % The first update occurs at 0.5 seconds.
    num_to_getvel = 20; % Obtain the average ODO velocity over 20 epochs.
end

if cfg.useZUPT                                         %%%
    zuptupdatetime = ceil(cfg.starttime) + 0.5;        %%%
end                                                    %%%

% data in process interval: Extract IMU/GNSS data based on the specified time interval.
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime, :);
gnssdata = gnssdata(gnssdata(:, 1) <= cfg.endtime, :);

%% for debug
disp("Start GNSS/INS Processing!");
lastprecent = 0;
odoscaleList = zeros(height(imudata),1);               %%%

%% initialization 
[kf, navstate] = Initialize(cfg);
laststate = navstate;
odoscaleList(1) = navstate.odoscale;                   %%%

% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);
gnssindex = 1;
while gnssdata(gnssindex, 1) < thisimu(1, 1)
    gnssindex = gnssindex + 1;
end

if cfg.useodonhc
    odoindex = 1;
    while ododata(odoindex, 1) < thisimu(1, 1) && odoindex < size(ododata, 1)
        odoindex = odoindex + 1;
    end
end

%% MAIN PROCEED PROCEDURE
for imuindex = 2:size(imudata, 1) - 1

    %% record updates
    gnssupdate = false;                         %%%
    odonhcupdate = false;                       %%%
    zuptupdate = false;                         %%%

    %% set value of last state
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    %% compensate IMU error
    thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    %% adjust GNSS index
    while (gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) < lastimu(1, 1)) 
        % The current IMU epoch lacks GNSS observations and has not reached the end.
        gnssindex = gnssindex + 1;
    end

    % check whether gnss data is valid
    if (gnssindex > size(gnssdata, 1))
        disp('GNSS file END!');
        break;
    end

    % test odo scale error: no gnss for 60s
    nognssforodo = cfg.testodoscale && imudata(imuindex,1)>3.791627427385435e+05 && imudata(imuindex,1)<3.792228111197277e+05;

    %% determine whether gnss update is required
    if lastimu(1, 1) == gnssdata(gnssindex, 1) && (~nognssforodo)
        %% do gnss update
        % do gnss update for the current state
        thisgnss = gnssdata(gnssindex, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);
        kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, lastimu, imudt ,cfg);
        [kf, navstate] = ErrorFeedback(kf, navstate, cfg);
        gnssindex = gnssindex + 1;
        laststate = navstate;
        
        % do propagation for current imu data
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu, cfg);
        kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime, cfg);

        gnssupdate = true;                  %%%

    elseif (lastimu(1, 1) <= gnssdata(gnssindex, 1) && thisimu(1, 1) > gnssdata(gnssindex, 1)) && (~nognssforodo)
        %% do gnss update
        % ineterpolate imu to gnss time
        [firstimu, secondimu] = interpolate(lastimu, thisimu, gnssdata(gnssindex, 1));
        
        % do propagation for first imu
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu, cfg);
        kf = InsPropagate(navstate, firstimu, imudt, kf, cfg.corrtime, cfg);

        % do gnss update
        thisgnss = gnssdata(gnssindex, :)';
        kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, firstimu, imudt ,cfg);
        [kf, navstate] = ErrorFeedback(kf, navstate, cfg);
        gnssindex = gnssindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu, cfg);
        kf = InsPropagate(navstate, secondimu, imudt, kf, cfg.corrtime, cfg);

        gnssupdate = true;                  %%%
        
    else
        %% only do propagation, no gnss update
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu, cfg);
        % error propagation
        kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime, cfg);

    end

    %% ZUPT update process (ZARU included)
    if cfg.useZUPT && useZUPT(navstate.time)                %%%%%%%%%%%%%%%
        %% reset imudt                                         ZUPT PART
        imudt = thisimu(1, 1) - lastimu(1, 1);              %%%%%%%%%%%%%%%

        %% adjust ZUPT timestamp
        if zuptupdatetime <= thisimu(1, 1)
            zuptupdatetime = thisimu(1, 1);
            %% ZUPT udpate
            kf = ZUPTUpdate(navstate, kf, cfg, thisimu, imudt);
            [kf, navstate] = ErrorFeedback(kf, navstate, cfg);
            
            zuptupdate = true;

            %% update zupt-update time
            zuptupdatetime = zuptupdatetime + 1 / cfg.zuptupdaterate;
        end
    end

    %% odonhc update process
    if cfg.useodonhc
        %% reset imudt
        imudt = thisimu(1, 1) - lastimu(1, 1); %%%
       
        %% update odo index
        while ododata(odoindex, 1) < thisimu(1, 1) && odoindex < size(ododata, 1)
            odoindex = odoindex + 1;
        end

        %% odonhc udpate
        if (lastimu(1, 1) <= odoupdatetime && odoupdatetime < thisimu(1, 1))
            startindex = odoindex - round(num_to_getvel / 2);
            endindex = odoindex + round(num_to_getvel / 2);
            if (startindex < 1)
                startindex = 1;
            end
            if (endindex > size(ododata, 1))
                endindex = size(ododata, 1);
            end
           
            % get odovel and update
            [odovel, valid] = getodovel(ododata(startindex:endindex, :), thisimu(1, 1));
            if valid
                odonhc_vel = [odovel; 0; 0];
                kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, imudt);
                [kf, navstate] = ErrorFeedback(kf, navstate, cfg);

                odonhcupdate = true;
            end
            odoupdatetime = odoupdatetime + 1 / cfg.odoupdaterate;
        end
    end
    
    %% save data
    % write navresult to file 
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);

    % write imu error
    imuerror = zeros(13, 1);
    imuerror(1, 1) = navstate.time;
    imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
    imuerror(5:7, 1) = navstate.accbias * 1e5;
    imuerror(8:10, 1) = navstate.gyrscale * 1e6;
    imuerror(11:13, 1) = navstate.accscale * 1e6;
    fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);

    % write state std
    std = zeros(1, 22);
    std(1) = navstate.time;
    for idx=1:21
        std(idx + 1) = sqrt(kf.P(idx, idx));
    end
    std(8:10) = std(8:10) * param.R2D;
    std(11:13) = std(11:13) * param.R2D * 3600;
    std(14:16) = std(14:16) * 1e5;
    std(17:22) = std(17:22) * 1e6;
    fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);

    %% print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.01) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end

    %% debug
    odoscaleList(imuindex) = navstate.odoscale;
end

% close file
fclose(imuerrfp);
fclose(navfp);
fclose(stdfp);

disp("GNSS/INS Integration Processing Finished!");


