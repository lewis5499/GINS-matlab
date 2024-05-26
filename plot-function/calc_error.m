format long;
disp("Start Loading reference result!")

%% load data （nav文件第一列为0，不需要用）
% temp = load('dataset/NavResult.nav');
temp = load('dataset/NavResult_ODONHC.nav');
result_all = temp(:, 2:end);
temp=load('dataset/REF_NAV.nav');
ref = temp(:, 2:end);

n2v = 0;

cfg = ProcessConfig();

%%  航向角平滑
for i=2:size(result_all, 1)
    if (result_all(i,10) - result_all(i-1, 10)) < -180
        result_all(i:end, 10) = result_all(i:end, 10) + 360;
    end
    if (result_all(i,10) - result_all(i-1, 10)) > 180
        result_all(i:end, 10) = result_all(i:end, 10) - 360;
    end
end

for i=2:size(ref, 1)
    if (ref(i,10) - ref(i-1, 10)) < -180
        ref(i:end, 10) = ref(i:end, 10) + 360;
    end
    if (ref(i,10) - ref(i-1, 10)) > 180
        ref(i:end, 10) = ref(i:end, 10) - 360;
    end
end

% figure()
% plot(result_all(:, 10))

%% 找到数据重合部分
res_start = result_all(1, 1);
res_end = result_all(end, 1);
ref_start = ref(1, 1);
ref_end = ref(end, 1);

if (ref_start > res_start)
    starttime = ref_start;
else 
    starttime = res_start;
end
if (res_end > ref_end)
    endtime = ref_end;
else 
    endtime = res_end;
end

% 按照采样间隔取合适的时间
dt = mean(diff(result_all(:, 1)));
time = starttime:dt:endtime;
time = time';


%% 测试结果和参考结果内插到同样的时刻，然后求差
newresult = zeros(size(time, 1), 10);
newref = zeros(size(time, 1), 10);
error = zeros(size(time, 1), 10);
newresult(:, 1) = time;
newref(:, 1) = time;
error(: ,1) = time;

newref(:, 2:10) = interp1(ref(:, 1), ref(:, 2:10), time);
newresult(:, 2:10) = interp1(result_all(:, 1), result_all(:, 2:10), time);
error(:, 2:10) = newresult(:, 2:10) - newref(:, 2:10);

% 航向角误差处理
for i = 1:size(error, 1)
    if error(i, 10) > 180
        error(i, 10) = error(i, 10) - 360;
    end
    if error(i, 10) < -180
        error(i, 10) = error(i, 10) + 360;
    end
end

%% 位置误差转到ned
D2R=pi/180.0;
R2D=180.0/pi;
first_blh = result_all(1, 2:4);
RM = getRm(first_blh(1) * D2R);
RN = getRn(first_blh(1) * D2R);
h = first_blh(2);
DR = diag([RM + h, (RN + h)*cos(first_blh(1) * D2R), -1]);

error(:, 2:3) = error(:, 2:3) * D2R;
for i = 1:size(error, 1)
    delta_pos = DR * (error(i, 2:4)');
    error(i, 2:4) = delta_pos';
end

%% n-frame to v-frame
% https://blog.csdn.net/scott198510/article/details/124562897
if n2v
    newref(:, 8:10) = newref(:, 8:10) * D2R;
    r = cfg.installangle(1);
    p = cfg.installangle(2);
    y = cfg.installangle(3);
    Rbv = [cos(y)*cos(p), -sin(y)*cos(r)+cos(y)*sin(p)*sin(r),  sin(y)*sin(r)+cos(y)*sin(p)*cos(r);
           sin(y)*cos(p),  cos(y)*cos(r)+sin(y)*sin(p)*sin(r), -cos(y)*sin(r)+sin(y)*sin(p)*cos(r);
              -sin(p),            sin(r)*cos(p),                          cos(r)*cos(p)          ];
    
    for i = 1:size(newref, 1)
        Cnb = rotMat('roll', newref(i, 8), 'left')*...
              rotMat('pitch', newref(i, 9), 'left')*...
              rotMat('yaw', newref(i, 10), 'left');
        Cnv = Rbv * Cnb;
        error(i, 5:7) = (Cnv * error(i, 5:7)')';
        error(i, 2:4) = (Cnv * error(i, 2:4)')';
    end

end

%% plot
figure;
set(gcf,'Position',[150 100 1050 700])
plot(error(:,1),error(:,2:4));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Position-Error}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{Error[m]}$','interpreter','latex','FontSize', 17) 

if n2v 
    legend1=legend('$\bf{xVehicle}$','$\bf{yVehicle}$','$\bf{zVehicle}$','interpreter','latex','FontSize',10.5); 
    set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
else
    legend1=legend('$\bf{North}$','$\bf{East}$','$\bf{Down}$','interpreter','latex','FontSize',10.5); 
    set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
end
box on
grid("on");

% 速度误差
figure;
set(gcf,'Position',[150 100 1050 700])
plot(error(:,1),error(:,5:7));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Velocity-Error}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{Error[m/s]}$','interpreter','latex','FontSize', 17) 

if n2v 
    legend1=legend('$\bf{xVehicle}$','$\bf{yVehicle}$','$\bf{zVehicle}$','interpreter','latex','FontSize',10.5); 
    set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
else
    legend1=legend('$\bf{North}$','$\bf{East}$','$\bf{Down}$','interpreter','latex','FontSize',10.5); 
    set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
end
box on
grid("on");

% 姿态误差
figure;
set(gcf,'Position',[150 100 1050 700])
plot(error(:,1),error(:,8:10));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Attitude-Error}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{Error[deg]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{Roll}$','$\bf{Pitch}$','$\bf{Yaw}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid("on");

% figure;
% subplot(3,1,1)
% title("ref")
% plot(newref(:,1),newref(:,5))
% subplot(3,1,2)
% plot(newref(:,1),newref(:,6))
% subplot(3,1,3)
% plot(newref(:,1),newref(:,7))
% 
% figure;
% subplot(3,1,1)
% title("ref")
% plot(newref(:,1),newref(:,8))
% subplot(3,1,2)
% plot(newref(:,1),newref(:,9))
% subplot(3,1,3)
% plot(newref(:,1),newref(:,10))
% 
% figure;
% subplot(3,1,1)
% title("result")
% plot(newresult(:,1),newresult(:,5))
% subplot(3,1,2)
% plot(newresult(:,1),newresult(:,6))
% subplot(3,1,3)
% plot(newresult(:,1),newresult(:,7))

