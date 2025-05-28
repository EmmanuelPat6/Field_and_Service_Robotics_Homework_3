%% Geometric Control

%% INIZIALIZATION
clc
clear
close all

%% Simulink Scheme
out = sim("geometric_control_template.slx");

%% Plot From Simulink
t_out = out.tout;

plot_1 = figure('Name', 'Position x and y Errors', 'Position', [10 10 900 350]);
plot(t_out,out.err_p.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.err_p.Data(:,2),'k--','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.err_p.Data(:,1); out.err_p.Data(:,2)])-0.00000001, max([out.err_p.Data(:,1); out.err_p.Data(:,2)])+0.00000001])
legend({'$e_{p_x}(t)$', '$e_{p_y}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_1, 'Position_x_y_Errors_Geometric_Control.pdf');



plot_2 = figure('Name', 'Position z Error', 'Position', [10 10 900 350]);
plot(t_out,out.err_p.Data(:,3),'k','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.err_p.Data(:,3)])-0.000001, max([out.err_p.Data(:,3)])+0.000001])
legend({'$e_{p_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_2, 'Position_z_Error_Geometric_Control.pdf');


plot_3 = figure('Name', 'Velocity x and y Errors', 'Position', [10 10 900 350]);
plot(t_out,out.dot_err_p.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.dot_err_p.Data(:,2),'k--','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.dot_err_p.Data(:,1); out.dot_err_p.Data(:,2)])-0.0000001, max([out.dot_err_p.Data(:,1); out.dot_err_p.Data(:,2)])+0.0000001])
legend({'$e_{\dot{p}_x}(t)$', '$e_{\dot{p}_y}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_3, 'Velocity_x_y_Errors_Geometric_Control.pdf');


plot_4 = figure('Name', 'Velocity z Error', 'Position', [10 10 900 350]);
plot(t_out,out.dot_err_p.Data(:,3),'k','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.dot_err_p.Data(:,3)])-0.00001, max([out.dot_err_p.Data(:,3)])+0.00001])
legend({'$e_{\dot{p}_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_4, 'Velocity_z_Error_Geometric_Control.pdf');

plot_5 = figure('Name', 'Angular x and y Errors', 'Position', [10 10 900 350]);
plot(t_out,out.err_R.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.err_R.Data(:,2),'k--','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.err_R.Data(:,1); out.err_R.Data(:,2)])-0.00000001, max([out.err_R.Data(:,1); out.err_R.Data(:,2)])+0.00000001])
legend({'$e_{R_1}(t)$', '$e_{R_2}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_5, 'Angular_1_2_Errors_Geometric_Control.pdf');



plot_6 = figure('Name', 'Angular z Error', 'Position', [10 10 900 350]);
plot(t_out,out.err_R.Data(:,3),'k','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.err_R.Data(:,3)])-0.000001, max([out.err_R.Data(:,3)])+0.000001])
legend({'$e_{R_3}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_6, 'Angular_3_Error_Geometric_Control.pdf');


plot_7 = figure('Name', 'Angular Velocity x and y Errors', 'Position', [10 10 900 350]);
plot(t_out,out.err_W.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.err_W.Data(:,2),'k--','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.err_W.Data(:,1); out.err_W.Data(:,2)])-0.000001, max([out.err_W.Data(:,1); out.err_W.Data(:,2)])+0.000001])
legend({'$e_{\omega_1}(t)$', '$e_{\omega_2}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_7, 'Angular_Velocity_1_2_Errors_Geometric_Control.pdf');


plot_8 = figure('Name', 'Angular Velocity z Error', 'Position', [10 10 900 350]);
plot(t_out,out.err_W.Data(:,3),'k','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.err_W.Data(:,3)])-0.000001, max([out.err_W.Data(:,3)])+0.000001])
legend({'$e_{\omega_3}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_8, 'Angular_Velocity_3_Error_Geometric_Control.pdf');


plot_9 = figure('Name', 'Thrust', 'Position', [10 10 900 350]);
plot(t_out,out.uT.Data,'k','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$F [N]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.uT.Data])-0.01, max([out.uT.Data])+0.01])
legend({'$u_T(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_9, 'uT_Geometric_Control.pdf');

plot_10 = figure('Name', 'Drag Moment x and y', 'Position', [10 10 900 350]);
plot(t_out,out.tau_b.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.tau_b.Data(:,2),'k--','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\tau_b [Nm]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.tau_b.Data(:,1); out.tau_b.Data(:,2)])-0.0001, max([out.tau_b.Data(:,1); out.tau_b.Data(:,2)])+0.0001])
legend({'$\tau_x(t)$', '$\tau_y(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_10, 'tau_b_x_y_Geometric_Control.pdf');



plot_11 = figure('Name', 'Drag Moment z', 'Position', [10 10 900 350]);
plot(t_out,out.tau_b.Data(:,3),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\tau_b [Nm]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.tau_b.Data(:,3)])-0.001, max([out.tau_b.Data(:,3)])+0.001])
legend({'$\tau_z(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_11, 'tau_b_z_Geometric_Control.pdf');

