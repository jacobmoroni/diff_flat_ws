function createfigure(XMatrix1, YMatrix1, ZMatrix1)
%CREATEFIGURE1(XMATRIX1, YMATRIX1, ZMATRIX1)
%  XMATRIX1:  matrix of x data
%  YMATRIX1:  matrix of y data
%  ZMATRIX1:  matrix of z data

%  Auto-generated by MATLAB on 06-Apr-2018 13:09:25

% Create figure
figure1 = figure(1);

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple lines using matrix input to plot3
plot31 = plot3(XMatrix1,YMatrix1,ZMatrix1,'Parent',axes1,'LineWidth',2);
set(plot31(1),'DisplayName','Command','LineStyle','--','Color',[0 0 0]);
set(plot31(2),'DisplayName','PID','Color',[0 0 1]);
set(plot31(3),'DisplayName','LQR','Color',[1 0 0]);
set(plot31(4),'DisplayName','PID only','Color',[1 1 0]);

% plot31 = plot3(XMatrix1,YMatrix1,ZMatrix1,'Parent',axes1);
% set(plot31(1),'DisplayName','Command');
% set(plot31(2),'DisplayName','With Differential Flatess');
% set(plot31(3),'DisplayName','PID only');

view(axes1,[-35 67]);
grid(axes1,'on');
zlim([-2.5,-1.5]);
title('Trajectory Over Time','fontsize',16);
xlabel('Pn','fontsize',12);
ylabel('Pe','fontsize',12);
zlabel('Pd','fontsize',12);
% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.141947550473154 0.873205126506273 0.20873354988746 0.0769230783130476],...
    'fontsize',12);

