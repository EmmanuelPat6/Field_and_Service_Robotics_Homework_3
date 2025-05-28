%% Tilting Control Voliro Approach

%% INIZIALIZATION
clc
clear
close all

%% Simulink Scheme
out = sim("tilting_control_voliro_template.slx");

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
ylim([min([out.err_p.Data(:,1); out.err_p.Data(:,2)])-0.00001, max([out.err_p.Data(:,1); out.err_p.Data(:,2)])+0.00001])
legend({'$e_{p_x}(t)$', '$e_{p_y}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_1, 'Position_x_y_Errors_Tilting_Control_Voliro.pdf');



plot_2 = figure('Name', 'Position z Error', 'Position', [10 10 900 350]);
plot(t_out,out.err_p.Data(:,3),'k.-','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.err_p.Data(:,3)])-0.0000001, max([out.err_p.Data(:,3)])+0.0000001])
legend({'$e_{p_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_2, 'Position_z_Error_Tilting_Control_Voliro.pdf');


plot_3 = figure('Name', 'Velocity Errors', 'Position', [10 10 900 350]);
plot(t_out,out.dot_err_p.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.dot_err_p.Data(:,2),'k--','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
plot(t_out,out.dot_err_p.Data(:,3),'k-.','Color', [0.6, 0.6, 0.6], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.dot_err_p.Data(:,1); out.dot_err_p.Data(:,2); out.dot_err_p.Data(:,3)])-0.00001, max([out.dot_err_p.Data(:,1); out.dot_err_p.Data(:,2); out.dot_err_p.Data(:,3)])+0.00001])
legend({'$e_{\dot{p}_x}(t)$', '$e_{\dot{p}_y}(t)$', '$e_{\dot{p}_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_3, 'Velocity_x_y_z_Errors_Tilting_Control_Voliro.pdf');


% plot_4 = figure('Name', 'Velocity z Error', 'Position', [10 10 900 350]);
% plot(t_out,out.dot_err_p.Data(:,3),'k.-','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
% xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
% ylabel('$e [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
% set(gca, 'FontSize', 26);
% grid on
% box on
% xlim([0, t_out(end)])
% ylim([min([out.dot_err_p.Data(:,3)])-0.000001, max([out.dot_err_p.Data(:,3)])+0.000001])
% legend({'$e_{\dot{p}_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'best')
% set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
% annotation('rectangle',[0 0 1 1],'Color','w');
% exportgraphics(plot_4, 'Velocity_z_Error_Tilting_Control_Voliro.pdf');

plot_5 = figure('Name', 'Angular Errors', 'Position', [10 10 900 350]);
plot(t_out,out.eR.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.eR.Data(:,2),'k--','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
plot(t_out,out.eR.Data(:,3),'k-.','Color', [0.6, 0.6, 0.6], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.eR.Data(:,1); out.eR.Data(:,2); out.eR.Data(:,3)])-0.0000001, max([out.eR.Data(:,1); out.eR.Data(:,2); out.eR.Data(:,3)])+0.0000001])
legend({'$e_{R_1}(t)$', '$e_{R_2}(t)$', '$e_{R_3}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_5, 'Angular_1_2_3_Errors_Tilting_Control_Voliro.pdf');



% plot_6 = figure('Name', 'Angular z Error', 'Position', [10 10 900 350]);
% plot(t_out,out.eR.Data(:,3),'k.-','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
% xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
% ylabel('$e [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
% set(gca, 'FontSize', 26);
% grid on
% box on
% xlim([0, t_out(end)])
% ylim([min([out.eR.Data(:,3)])-0.000001, max([out.eR.Data(:,3)])+0.000001])
% legend({'$e_{r_3}(t)$'}, 'Interpreter', 'latex', 'Location', 'best')
% set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
% annotation('rectangle',[0 0 1 1],'Color','w');
% exportgraphics(plot_6, 'Angular_3_Error_Tilting_Control_Voliro.pdf');


plot_7 = figure('Name', 'Angular Velocity Errors', 'Position', [10 10 900 350]);
plot(t_out,out.ew.Data(:,1),'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.ew.Data(:,2),'k--','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
plot(t_out,out.ew.Data(:,3),'k-.','Color', [0.6, 0.6, 0.6], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.ew.Data(:,1); out.ew.Data(:,2); out.ew.Data(:,3)])-0.0000001, max([out.ew.Data(:,1); out.ew.Data(:,2); out.ew.Data(:,3)])+0.0000001])
legend({'$e_{\omega_1}(t)$', '$e_{\omega_2}(t)$', '$e_{\omega_3}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_7, 'Angular_Velocity_1_2_3_Errors_Tilting_Control_Voliro.pdf');


% plot_8 = figure('Name', 'Angular Velocity z Error', 'Position', [10 10 900 350]);
% plot(t_out,out.ew.Data(:,3),'k-.','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
% xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
% ylabel('$e [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
% set(gca, 'FontSize', 26);
% grid on
% box on
% xlim([0, t_out(end)])
% ylim([min([out.ew.Data(:,3)])-0.000001, max([out.ew.Data(:,3)])+0.000001])
% legend({'$e_{\omega_3}(t)$'}, 'Interpreter', 'latex', 'Location', 'best')
% set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
% annotation('rectangle',[0 0 1 1],'Color','w');
% exportgraphics(plot_8, 'Angular_Velocity_3_Error_Tilting_Control_Voliro.pdf');

plot_9 = figure('Name', 'Rotor Speed', 'Position', [10 10 900 350]);
plot(t_out,out.uw.Data(:,1),'k','Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
hold on
plot(t_out,out.uw.Data(:,2),'k--','Color', [0.3, 0.3, 0.3], 'LineWidth', 2.5)
plot(t_out,out.uw.Data(:,3),'k-.','Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
plot(t_out,out.uw.Data(:,4),'k','Color', [0.6, 0.6, 0.6], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$u_\omega [rad^2/s^2]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.uw.Data(:,1); out.uw.Data(:,2); out.uw.Data(:,3); out.uw.Data(:,4)])-10000, max([out.uw.Data(:,1); out.uw.Data(:,2); out.uw.Data(:,3); out.uw.Data(:,4)])+10000])
legend({'$u_{\omega_1}(t)$','$u_{\omega_2}(t)$','$u_{\omega_3}(t)$','$u_{\omega_4}(t)$'}, 'Interpreter', 'latex', 'Location', 'northeastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_9, 'uw_Tilting_Control_Voliro.pdf');


plot_10 = figure('Name', 'Rotor Orientation', 'Position', [10 10 900 350]);
plot(t_out,out.alfa.Data(:,1),'Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5)
hold on
plot(t_out,out.alfa.Data(:,2),'k--','Color', [0.3, 0.3, 0.3], 'LineWidth', 2.5)
plot(t_out,out.alfa.Data(:,3),'k-.','Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
plot(t_out,out.alfa.Data(:,4),'k','Color', [0.6, 0.6, 0.6], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\alpha [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, t_out(end)])
ylim([min([out.alfa.Data(:,1); out.alfa.Data(:,2); out.alfa.Data(:,3); out.alfa.Data(:,4)])-0.0001, max([out.alfa.Data(:,1); out.alfa.Data(:,2); out.alfa.Data(:,3); out.alfa.Data(:,4)])+0.0001])
legend({'$\alpha_1(t)$', '$\alpha_2(t)$', '$\alpha_3(t)$', '$\alpha_4(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_10, 'alfa_Tilting_Control_Voliro.pdf');
