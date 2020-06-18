clc
clear all
close all
global G N weight c theta_d u_d

u_d = 0.9;
theta_d = pi/4;

weight = 0.2;

% c is the desired squared of the distance between the unicycles
des_dist = 12;
c = des_dist^2;

% total simulation time
t_simulation = 20;

% First I need to define the balanced and strongly connected graph
% I build the Graph defining first the adjacency matrix A

A = [0 0 1 0;...
    0 0 0 1;...
    0 1 0 0;...
    1 0 0 0];
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
theta_2_0 = -pi/2;
x_3_0 = 10;
y_3_0 = 8;
theta_3_0 = 2*pi/3;
x_4_0 = -1;
y_4_0 = -15;
theta_4_0 = pi;


initialconditions=[x_1_0,y_1_0,theta_1_0,x_2_0,y_2_0,...
    theta_2_0,x_3_0,y_3_0,theta_3_0,...
    x_4_0,y_4_0,theta_4_0]';
TSPAN=[0 t_simulation];
tol=1e-3;
options=odeset('RelTol',tol,'AbsTol',[tol tol tol tol tol...
    tol tol tol tol tol tol tol]);

% Rendezvous control
[t,x]=ode23t('consensus_control_unicycle',TSPAN,...
    initialconditions,options);

tol=1e-3;
options=odeset('RelTol',tol,'AbsTol',[tol tol tol tol tol...
    tol tol tol tol tol tol tol]);
% Formation control
[t_for,x_for] = ode23t('formation_control',TSPAN,...
    initialconditions,options);


% Signal for Simulink 3D

sig.time = t_for;
sig.signals.values = x_for;
sig.signals.dimensions = N*3;

% Plot positions, rendezvous
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
    title("Formation control")
    plot(x_i,y_i,'-o','LineWidth',1.6,'MarkerSize',7,'MarkerIndices',1);
    xlabel("Position x [m]")
    ylabel("Position y [m]")
    xlim([-13 26])
    ylim([-20 20])
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
    xlim([0 7])
    hold on
end