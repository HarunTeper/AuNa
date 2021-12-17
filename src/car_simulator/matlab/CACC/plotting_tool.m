clf(figure(1));

set(0,'DefaultAxesTickLabelInterpreter','latex'); 
set(0,'DefaultTextInterpreter','latex'); 
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesFontSize',16);
set(0,'DefaultLegendFontSize',8);

%line width
LW = 1.8;

figure(1);
hold on;

row = 1;

plot(TX.Time,TX.Data(:,row)/100,'LineWidth',LW)
plot(RX.Time-0.17,RX.Data(:,row)/100,'LineWidth',LW)
grid; xlabel('t [s]'); ylabel('v [m/s]');