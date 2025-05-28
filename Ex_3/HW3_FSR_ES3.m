% Let us implement a MOMENUTM-BASED ESTIMATOR of order r to estimate the
% external disturbances acting on the UAV during the flight using the
% workspace file attached as ws_homework_3_2025.mat. Furthermore, let us
% suppose that the sampling time of the esitamtor is equal to 1ms and let
% us compare the obtained estimation with the following disturbances
% applied during the flight:
% 1 N along the x and y axis of the World Frame
% -0.4 Nm around the Yaw Axis
% The quadroto ahs a supposed mass of 1.5 Kg and an Inertia Matrix referred
% to the Body Frame equal to diag([1.2416 1.2416 2*1.2416])

%% INIZIALIZATION
clc
clear
close all

%% Flight Values
load('ws_homework_3_2025.mat')

%% Quadrotor Parameters
m_s = 1.5;    % Supposed Mass
Ib = diag([1.2416 1.2416 2*1.2416]);    % Inertia Matrix referred to the Body Frame
g = 9.81;   % Gravity Acceleration
e3 = [0 0 1]';

%% Disturbances
Dxy = 1;    % Along x- and y-axis 
Dyaw = -0.4;    % Around Yaw Axis

%% Data from the Workspace
uT = thrust.signals.values; % Commandend Thrust
taub = tau.signals.values;  % Commanded Torques
pb_dot = linear_vel.signals.values; % Measured Linear Velocity
etab = attitude.signals.values; % Attitude expressed as Euler Angles (RPY)
etab_dot = attitude_vel.signals.values; % Attitude Derivative

t = attitude.time;  % Time

%% Momentum-Based Estimator
Ts = 10^-3; % Sampling Time of 1ms
r = 5;  % Estimator Order

c0 = 10; % The higher as possible
k0 = c0;    % k0/c0 = 1
s = tf('s');
G = k0^r/(s+c0)^r;  % rth-Order Transfer Function

% Ts_target = 2;  % Settling Time
% wc = 1.5*r/Ts_target;  % To obtain a Constant Settling Time for different order r
% G = wc^r/(s+wc)^r;

% Variables Inizialization
q = zeros(6,length(t));
gamma = zeros(6,length(t),r);
wrench_hat = zeros(6,length(t));
K = zeros(r,1);
c = cell2mat(G.Denominator);    % G.Denominator is a cell array with the
                    % vector of the coefficients of the denominator of G
c = c(2:end);   % No s^r coefficient which is always 1
                % In this way the first elementi is c_{r-1}

% Ki Computation
aux = 1;    % Auxiliary Variable
% From 1 to r (it is like from 0 to r-1 in the formula)
for j = 1:r
    % prod_{i=j+1}^{r} = c(j)
    K(j) = c(j)/aux;
    aux = aux*K(j);
end

K = flip(K);    % Reverse the order because c(1)=c_{r-1}

% RPY Angles
phi = etab(:,1);
theta = etab(:,2);
psi = etab(:,3);

% RPY Angles Time Derivative
phi_dot = etab_dot(:,1);
theta_dot = etab_dot(:,2);
psi_dot = etab_dot(:,3);

for i = 1:length(t)-1

    % SO(3) Attitude -> Non Minimal Representation from RPY (Euler Angles) to Rb
    Rb = [cos(theta(i))*cos(psi(i)), sin(phi(i))*sin(theta(i))*cos(psi(i)) - cos(phi(i))*sin(psi(i)), cos(phi(i))*sin(theta(i))*cos(psi(i)) + sin(phi(i))*sin(psi(i));
         cos(theta(i))*sin(psi(i)), sin(phi(i))*sin(theta(i))*sin(psi(i)) + cos(phi(i))*cos(psi(i)), cos(phi(i))*sin(theta(i))*sin(psi(i)) - sin(phi(i))*cos(psi(i));
         -sin(theta(i)), sin(phi(i))*cos(theta(i)), cos(phi(i))*cos(theta(i))];
    
    % Transformation Matrix
    Q = [1, 0, -sin(theta(i)); 
        0, cos(phi(i)), cos(theta(i))*sin(phi(i));
        0, -sin(phi(i)), cos(theta(i))*cos(phi(i))];
        
    % Time Derivative
    Q_dot = [0, 0, -cos(theta(i))*theta_dot(i);
           0, -sin(phi(i))*phi_dot(i), -sin(theta(i))*sin(phi(i))*theta_dot(i) + cos(theta(i))*cos(phi(i))*phi_dot(i);
           0, -cos(phi(i))*phi_dot(i), -sin(theta(i))*cos(phi(i))*theta_dot(i) - cos(theta(i))*sin(phi(i))*phi_dot(i)];

    M = Q'*Ib*Q;
    
    S_Q_etab_dot = skew(Q*etab_dot(i,:)');
    
    % Coriolis-Like Matrix Computation
    C = Q'*S_Q_etab_dot*Ib*Q + Q'*Ib*Q_dot;

    % Momentum vector q Computation
    q(:,i+1) = [m_s*eye(3), zeros(3,3); zeros(3,3), M]*[pb_dot(i+1,:)';etab_dot(i+1,:)'];
        
    % Gamma 1 Computation
    gamma(:,i+1,1) = gamma(:,i,1) + K(1)*(q(:,i+1)-q(:,i) - Ts*(wrench_hat(:,i) + [m_s*g*e3 - uT(i)*Rb*e3; C'*etab_dot(i,:)' + Q'*taub(i,:)']));

    % Gamma Computation
    for j = 2:r
        gamma(:,i+1,j) = gamma(:,i,j) + K(j)*Ts*(-wrench_hat(:,i) + gamma(:,i,j-1));
    end

    % Estimation Update
    wrench_hat(:,i+1) = gamma(:,i+1,r);
end


%% Plot
% Remember that there are two supposing disturbances during the flight:
% 1 N along the x- and y- axis of the World Frame
% -0.4 Nm around the Yaw Axis

plot_1 = figure('Name', 'f_x_hat and disturbance', 'Position', [10 10 900 350]);
plot(t,wrench_hat(1,:),'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
hold on
plot(t,Dxy.*ones(length(t),1),'k--','Color',[0.8, 0.8, 0.8], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$F[N]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(1,:))-0.1, max(wrench_hat(1,:))+0.1])
legend({'$\hat{f}_x$', 'x-Disturbance'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_1, 'f_hat_x_and_disturbace.pdf');

plot_2 = figure('Name', 'f_y_hat and disturbance', 'Position', [10 10 900 350]);
plot(t,wrench_hat(2,:),'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
hold on
plot(t,Dxy.*ones(length(t),1),'k--','Color',[0.8, 0.8, 0.8], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$F[N]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(2,:))-0.1, max(wrench_hat(2,:))+0.1])
legend({'$\hat{f}_y$', 'y-Disturbance'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_2, 'f_hat_y_and_disturbace.pdf');

plot_3 = figure('Name', 'f_z_hat', 'Position', [10 10 900 350]);
plot(t,wrench_hat(3,:),'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$F[N]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(3,:))-0.1, max(wrench_hat(3,:))+0.1])
legend({'$\hat{f}_z$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_3, 'f_hat_z.pdf');

plot_4 = figure('Name', 'tau_x_hat', 'Position', [10 10 900 350]);
plot(t,wrench_hat(4,:),'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$\tau[Nm]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(4,:))-0.01, max(wrench_hat(4,:))+0.01])
legend({'$\hat{\tau}_x$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_4, 'tau_hat_x.pdf');

plot_5 = figure('Name', 'tau_y_hat', 'Position', [10 10 900 350]);
plot(t,wrench_hat(5,:),'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$\tau[Nm]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(5,:))-0.01, max(wrench_hat(5,:))+0.01])
legend({'$\hat{\tau}_y$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_5, 'tau_hat_y.pdf');

plot_6 = figure('Name', 'tau_z_hat and disturbance', 'Position', [10 10 900 350]);
plot(t,wrench_hat(6,:),'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
hold on
plot(t,Dyaw.*ones(length(t),1),'k--','Color',[0.8, 0.8, 0.8], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$\tau[Nm]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(6,:))-0.01, max(wrench_hat(6,:))+0.01])
legend({'$\hat{\tau}_z$', 'Yaw-Disturbance'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_6, 'tau_hat_z_and_disturbance.pdf');


%% Real Mass Computation

% From Steady State Equation
uT_z = uT(end)*Rb*e3;
m_r = uT_z(3)/g

% From the Estimated Disturbance along the z-axis
m_tilde = wrench_hat(3,end)/g;
m = m_s + m_tilde


%% Functions

% SO(3) Attitude -> Non Minimal Representation from RPY (Euler Angles) to Rb
% function Rb = rot_mat_attitude(phi, theta, psi)
% 
%     Rb = [cos(theta*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
%         cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
%         -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
% 
% end

% Skew Symmetric Operator (3x3 Matrix) Computation
function S = skew(omega)

    S = [   0       -omega(3)   omega(2);
        omega(3)     0       -omega(1);
        -omega(2)  omega(1)     0     ];

end

% Q and Qdot Computation
% function [Q, Q_dot] = transformation_matrix(phi, theta, phi_dot, theta_dot)
% 
%         Q = [1, 0, -sin(theta); 
%             0, cos(phi), cos(theta)*sin(phi);
%             0, -sin(phi), cos(theta)*cos(phi)];
% 
%         % Time Derivative
%         Q_dot = [0, 0, -cos(theta)*theta_dot;
%                 0, -sin(phi)*phi_dot, -sin(theta)*sin(phi)*theta_dot + cos(theta)*cos(phi)*phi_dot;
%                 0, -cos(phi)*phi_dot, -sin(theta)*cos(phi)*theta_dot - cos(theta)*sin(phi)*phi_dot];
% 
% end
% 




