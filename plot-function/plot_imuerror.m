% -------------------------------------------------------------------------
% Author: Liqiang Wang, 
% GNSS Research Center, Wuhan University, China.;
% Contact: wlq@whu.edu.cn;
% Date: 2022.11.30;
% -------------------------------------------------------------------------

%% imuerror
imuerrorfile = 'dataset/ImuError.txt';
err = load(imuerrorfile);


figure()
set(gcf,'Position',[150 100 1050 700])
plot(err(:, 1), err(:, 2:4))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{GyroBias}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{gb[deg/h]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(err(:, 1), err(:, 5:7))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{AccelBias}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{ab[mGal]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(err(:, 1), err(:, 8:10))
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{GyroScale}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{gs[ppm]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");

figure()
set(gcf,'Position',[150 100 1050 700])
plot(err(:, 1), err(:, 11:13));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{AccelScale}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{as[ppm]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{X}$','$\bf{Y}$','$\bf{Z}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");




