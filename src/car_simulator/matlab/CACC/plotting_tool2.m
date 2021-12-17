clf(figure(1));clf(figure(2));clf(figure(3));clf(figure(4));

start = 45;
cutoff = 2300;

set(0,'DefaultAxesTickLabelInterpreter','latex'); 
set(0,'DefaultTextInterpreter','latex'); 
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesFontSize',16);
set(0,'DefaultLegendFontSize',8);

%line width
LW = 1.8;

figure(1);
hold on;

plot(robot0.x.Data(start:cutoff),robot0.y.Data(start:cutoff),'LineWidth',LW)
plot(robot1.signal1.x.Data(start:cutoff),robot1.signal1.y.Data(start:cutoff),'LineWidth',LW)
plot(robot2.signal1.x.Data(start:cutoff),robot2.signal1.y.Data(start:cutoff),'LineWidth',LW)
plot(robot3.signal1.x.Data(start:cutoff),robot3.signal1.y.Data(start:cutoff),'LineWidth',LW)
grid; xlabel('x [m]'); ylabel('y [m]');


figure(2);
hold on;

plot(robot0.v.Data(start:cutoff),'LineWidth',LW)
plot(robot1.signal1.v.Data(start:cutoff),'LineWidth',LW)
plot(robot2.signal1.v.Data(start:cutoff),'LineWidth',LW)
plot(robot3.signal1.v.Data(start:cutoff),'LineWidth',LW)
grid; ylabel('v [m/s]','Interpreter','latex'); xlabel('time [s]','Interpreter','latex');

figure(3);
hold on;

plot(robot0.thetadot.Data(start:cutoff),'LineWidth',LW)
plot(robot1.signal1.thetadot.Data(start:cutoff),'LineWidth',LW)
plot(robot2.signal1.thetadot.Data(start:cutoff),'LineWidth',LW)
plot(robot3.signal1.thetadot.Data(start:cutoff),'LineWidth',LW)
grid; ylabel('omega [radian/s]','Interpreter','latex'); xlabel('time [s]','Interpreter','latex');

figure(4);
hold on;

dist1 = sqrt((robot1.signal1.x.Data(start:cutoff)-robot1.signal2.Data(start:cutoff,1)).^2+(robot1.signal1.y.Data(start:cutoff)-robot1.signal2.Data(start:cutoff,2)).^2);
dist2 = sqrt((robot2.signal1.x.Data(start:cutoff)-robot2.signal2.Data(start:cutoff,1)).^2+(robot2.signal1.y.Data(start:cutoff)-robot2.signal2.Data(start:cutoff,2)).^2);
dist3 = sqrt((robot3.signal1.x.Data(start:cutoff)-robot3.signal2.Data(start:cutoff,1)).^2+(robot3.signal1.y.Data(start:cutoff)-robot3.signal2.Data(start:cutoff,2)).^2);


plot(dist1,'LineWidth',LW)
plot(dist2,'LineWidth',LW)
plot(dist3,'LineWidth',LW)
ylabel('spacing distance [m]','Interpreter','latex'); xlabel('time [s]','Interpreter','latex');

save('scenario_one_new.mat','robot0','robot1','robot2','robot3','robot1input');