% -------------------------------------------------------------------------
% Author: Liqiang Wang, 
% GNSS Research Center, Wuhan University, China.;
% Contact: wlq@whu.edu.cn;
% Date: 2022.11.30;
% -------------------------------------------------------------------------


%navpath = "dataset/NavResult.nav";
navpath = "dataset/NavResult_ODONHC.nav";
navdata = load(navpath);

% velocity
figure()
set(gcf,'Position',[150 100 1050 700])
plot(navdata(:, 2), navdata(:, 6:8));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Velocity}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{vel[m/s]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{North}$','$\bf{East}$','$\bf{Down}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");

% attitude
figure()
set(gcf,'Position',[150 100 1050 700])
plot(navdata(:, 2), navdata(:, 9:11));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Attitude}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{Att[deg]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{Roll}$','$\bf{Pitch}$','$\bf{Yaw}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");

% position
D2R=pi/180.0;
R2D=180.0/pi;
blh = navdata(:, 3:5);
blh(:, 1) = blh(:, 1) * D2R;
blh(:, 2) = blh(:, 2) * D2R;
first_blh = blh(1, 1:3);

RM = getRm(first_blh(1));
RN = getRn(first_blh(1));
h = first_blh(2);
DR = diag([RM + h, (RN + h)*cos(first_blh(1)), -1]);

% blh to ned
pos = zeros(size(blh));
for i = 1:size(pos, 1)
    delta_blh = blh(i, :) - first_blh;
    delta_pos = DR * delta_blh';
    pos(i, :) = delta_pos';
end

%% plane position
figure()
set(gcf,'Position',[150 100 1050 700])
plot(pos(:, 2), pos(:, 1));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Position}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{East[m]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{North[m]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{pos}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");

%% height
figure()
set(gcf,'Position',[150 100 1050 700])
plot(navdata(:, 2), navdata(:, 5));
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
title({'$\bf{Height}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{Time[s]}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{Height[m]}$','interpreter','latex','FontSize', 17) 
legend1=legend('$\bf{height}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on 
grid("on");





