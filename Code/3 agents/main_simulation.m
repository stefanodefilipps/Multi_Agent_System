clc
clear all
close all

global G N weight c theta_d u_d

% Desired steady-state velocity
u_d = 0;

% Desired steady-state angle
theta_d = 0;

% Weight to reduce control effort
weight = 1;

% Desired distance between agents
des_dist = 7;
c = des_dist^2;

% Total simulation time
t_simulation = 15;

% First I need to define the balanced and strongly connected graph
% I build the Graph defining first the adjacency matrix A

A = [0 1 1;...
    1 0 1;...
    1 1 0];
G = digraph(A);

% Incidence matrix
E = incidence(G);

% Laplacian matrix
L = E*E';

% Check 2 eigenvalues in order to understand how the plots varies witch the
% strength of the graph connectivity
lambdas = eig(L);
lambda_2 = lambdas(2);

figure
plot(G,'LineWidth',1.5)
title('Agents interconnection')

N = size(A,1);

% Initial conditions
x_1_0 = -10;
y_1_0 = 12;
theta_1_0 = pi/2;
x_2_0 = 12;
y_2_0 = -12;
theta_2_0 = pi/2;
x_3_0 = 10;
y_3_0 = 8;
theta_3_0 = 0;

initialconditions=[x_1_0,y_1_0,theta_1_0,x_2_0,...
    y_2_0,theta_2_0,x_3_0,y_3_0,theta_3_0]';
TSPAN=[0 t_simulation];
tol=1e-3;
options=odeset('RelTol',tol,'AbsTol',[tol tol tol tol tol...
    tol tol tol tol]);

% Rendezvous control
[t,x]=ode23t('consensus_control_unicycle',TSPAN,...
    initialconditions,options);
tol=1e-3;

% Formation control
options=odeset('RelTol',tol,'AbsTol',[tol tol tol tol tol...
    tol tol tol tol]);
[t_for,x_for] = ode23t('formation_control',TSPAN,...
    initialconditions,options);

% Signal for Simulink 3D
sig.time = t_for;
sig.signals.values = x_for;
sig.signals.dimensions = N*3;

%Plot x vs y - Rendezvous
figure
for i=1:N
    x_i = x(:,3*i-2);
    y_i = x(:,3*i-1);
    plot(x_i,y_i,'-o','LineWidth',1.6,'MarkerSize',7,'MarkerIndices',1);
    title("Rendezvous")
    xlabel("Position x [m]")
    ylabel("Position y [m]")
    xlim([-13 15])
    hold on
end


%Plot x vs y - Formation
figure
for i=1:N
    x_i = x_for(:,3*i-2);
    y_i = x_for(:,3*i-1);
    title("Rendezvous keeping distance")
    plot(x_i,y_i,'-o','LineWidth',1.6,'MarkerSize',7,'MarkerIndices',1);
    xlabel("Position x [m]")
    ylabel("Position y [m]")
    xlim([-13 15])
    hold on;
end

% Plot headings for rendezvous

figure
for i=1:N
    theta_i = x(:,3*i);
    plot(t,theta_i,'LineWidth',1);
    title("Headings")
    xlabel("Time [s]")
    ylabel("\theta [rad]")
    %xlim([0 60])
    hold on
end

% Plot heading for formation
figure
for i=1:N
    theta_i = x_for(:,3*i);
    plot(t_for,theta_i,'LineWidth',1);
    title("Headings")
    xlabel("Time [s]")
    ylabel("\theta [rad]")
    hold on
end

% Plot control effort for rendezvous
figure()
subplot(2,1,1)
control = compute_control_rendezvous(t,x);
for i=1:N
    u_i = control(:,2*i-1);
    plot(t,u_i);
    title('Control effort')
    ylabel('$u_i$ [m/s]','Interpreter','Latex')
    xlabel('Time [s]')
    xlim([0 50])
    hold on
end

subplot(2,1,2)
for i=1:N
    omega_i = control(:,2*i);
    plot(t,omega_i);
    ylabel('$\omega_i$ [rad/s]','Interpreter','Latex')
    xlabel('Time [s]')
    %xlim([0 15])
    hold on
end

% Plot control effort for formation control
figure()
subplot(2,1,1)
control = compute_control_formation(t_for,x_for);
for i=1:N
    u_i = control(:,2*i-1);
    plot(t_for,u_i,'LineWidth',1.5);
    title('Control effort')
    ylabel('$u_i$ [m/s]','Interpreter','Latex')
    xlabel('Time [s]')
    xlim([0 2.5])
    hold on
end

subplot(2,1,2)
for i=1:N
    omega_i = control(:,2*i);
    plot(t_for,omega_i,'LineWidth',1.5);
    ylabel('$\omega_i$ [rad/s]','Interpreter','Latex')
    xlabel('Time [s]')
    xlim([0 5])
    hold on
end