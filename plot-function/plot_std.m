% -------------------------------------------------------------------------
% Author: Liqiang Wang, 
% GNSS Research Center, Wuhan University, China.;
% Contact: wlq@whu.edu.cn;
% Date: 2022.11.30;
% -------------------------------------------------------------------------


%% std
stdfile = 'dataset/NavSTD.txt';
std = load(stdfile);

figure()
set(gcf,'Position',[150 100 1050 700])
plot(std(:, 1), std(:, 2:4))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Position-STD}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{pos[m]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(std(:, 1), std(:, 5:7))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Velocity-STD}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{vel[m/s]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");


figure()
set(gcf,'Position',[150 100 1050 700])
plot(std(:, 1), std(:, 8:10))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Attitude-STD}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{att[deg]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(std(:, 1), std(:, 11:13))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{GyroBias-STD}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{gb[deg/h]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(std(:, 1), std(:, 14:16))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{AccelBias-STD}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{ab[mGal]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(std(:, 1), std(:, 17:19))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{GyroScale-STD}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{gs[ppm]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(std(:, 1), std(:, 20:22));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{AccelScale-STD}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{as[ppm]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid("on");



