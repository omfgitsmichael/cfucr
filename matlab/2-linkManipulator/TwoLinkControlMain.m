%% 2-Link Robotic Arm Dynamics and Control %%
close all; clear all; clc;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);
enable_dynamics_test = false;

%% Time for Simulations %%
T0 = 0; Tf = 10; Delt = 0.01;
Tvec = linspace(T0,Tf,Tf/Delt);

%% Declare Global Variables %%
global enable_gravity enable_g_terms enable_robust enable_adaptive;
global lambda K gamma max_torque rho epsilon globalP;

% Enable certain fields %
enable_gravity = true;
enable_g_terms = true;

% Can only use robust or adaptive, cannot use both %
% Robust Control Does not work at the moment %
enable_robust = false;
enable_adaptive = true;

% Linear Control Gains - Must be Diagonal Matrices %
lambda = [3.5355 0; 0 3.5355]*3.5;
K = [7.071 0; 0 7.071]*3.5;

% Maximum allowable torque from motors %
max_torque = 15;

% Rate of Adaptivity
if enable_g_terms
    gamma = [5 0 0 0 0; 0 5 0 0 0; 0 0 5 0 0; 0 0 0 0.02 0; 0 0 0 0 0.02];
else
    gamma = [5 0 0; 0 5 0; 0 0 5];
end

% Maximum unmodeled dynamics to be considered as uncertainty %
if enable_robust && ~enable_adaptive
    rho = 5.0;
    epsilon = 0.001;
end

%% Desired Path %%
qdes1 = acos(0.45-0.45*sin(5*Tvec+pi/2));
qdes2 = -2*qdes1;

qdot_des1 = -2.25*sin(5*Tvec)./sqrt(1-0.2025*(1-cos(5*Tvec)).^2);
qdot_des2 = -2*qdot_des1;

qddot_des1 = -11.25*cos(5*Tvec)./sqrt(1-0.2025*(1-cos(5*Tvec)).^2)-...
    (2.27813*(sin(5*Tvec).^2).*(1-cos(5*Tvec)))/...
    (1-0.2025*(1-cos(5*Tvec)).^2).^(3/2);
qddot_des2 = -2*qddot_des1;

qdes1 = qdes1/4;
qdes2 = qdes2/4;

qdot_des1 = qdot_des1/4;
qdot_des2 = qdot_des2/4;

qddot_des1 = qddot_des1/4;
qddot_des2 = qddot_des2/4;

%% Initial parameters and estimates %%
p1 = 0.3;
p2 = 0.2;
p3 = 0.1;

if enable_gravity
    p4 = 6.0;
    p5 = 3.0;
end

if enable_robust && ~enable_adaptive
    globalP = [p1; p2; p3];
    
    if enable_gravity
        globalP = [p1; p2; p3; p4; p5];
    end
end
    
th1 = qdes1(1)+(5*pi/180);
th2 = qdes2(1)+(5*pi/180);
thd1 = qdot_des1(1)+(2*pi/180);
thd2 = qdot_des2(1)+(2*pi/180);

if enable_adaptive
    x02 = [th1; th2; thd1; thd2; p1; p2; p3];

    if enable_gravity && enable_g_terms
        x02 = [th1; th2; thd1; thd2; p1; p2; p3; p4; p5];
    end
end

if enable_robust
    x02 = [th1; th2; thd1; thd2];
end

%% Nominal Parameters %%
m1 = 2; % Mass of link 1 (kg) %
m2 = 2; % Mass of link 2 (kg) %
lc1 = 0.15; % Length of link 1 (meters) %
l1 = 0.3; % Length of link 1 (meters) %
lc2 = 0.15; % Length of link 1 (meters) %
l2 = 0.3; % Length of link 2 (meters) %

I1 = (1/3)*m1*l1^2;
I2 = (1/3)*m2*l2^2;

%% Robotic Arm Dynamics %%
if enable_dynamics_test
    
    % Initial Conditions %
    theta1 = pi/3; % Rad - Arm %
    theta2 = pi/8; % Rad - Arm %
    omega1 = pi/12; % Rad/Sec - Arm %
    omega2 = pi/12; % Rad/Sec - Arm %

    x0 = [theta1; theta2; omega1; omega2];

    [T,X] = ode45(@(t,x)TwoLinkDynamics(t,x,m1,m2,lc1,l1,lc2,I1,I2,true)...
        ,Tvec,x0,options);
end

%% True Parameters %%
p1true = 0.24;
p2true = 0.06;
p3true = 0.09;

p = [p1true; p2true; p3true];

if enable_gravity
    p4true = (m1*lc1+m2*l1)*9.81;
    p5true = m2*lc2*9.81;
    
    p = [p1true; p2true; p3true; p4true; p5true];
end

qd1 = [qdes1' qdot_des1' qddot_des1'];
qd2 = [qdes2' qdot_des2' qddot_des2'];

%% Robotic Arm Control %%
[T2,X2] = ode45(@(t,x)TwoLinkDynamicsAndControl(t,x,p,Tvec,qd1,qd2,'passivity')...
    ,Tvec,x02,options);

%% Graphs and Simulations %%
if enable_dynamics_test
    figure(1)
    subplot 211
    hold on
    plot(T,X(:,1))
    plot(T,X(:,2))
    hold off;
    legend('\theta_1','\theta_2')
    grid on; title('\theta Over Time')
    subplot 212
    hold on;
    plot(T,X(:,3))
    plot(T,X(:,4))
    hold off;
    legend('\omega_1','\omega_2')
    grid on; title('\omega Over Time')
end

figure(2)
subplot 211
hold on
plot(T2,X2(:,1))
plot(T2,X2(:,2))
plot(T2, qd1(:,1))
plot(T2, qd2(:,1))
hold off;
legend('\theta_1','\theta_2','\theta_1_{des}','\theta_2_{des}')
grid on; title('Controlled \theta Output Over Time')
subplot 212
hold on;
plot(T2,X2(:,3))
plot(T2,X2(:,4))
plot(T2, qd1(:,2))
plot(T2, qd2(:,2))
hold off;
legend('\omega_1','\omega_2','\omega_1_{des}','\omega_2_{des}')
grid on; title('Controlled \omega Output Over Time')

figure(3)
hold on
plot(T2,X2(:,5))
plot(T2,X2(:,6))
plot(T2,X2(:,7))
hold off;
legend('p_1','p_2','p_3')
grid on; title('Parameter Estimates Over Time')

if enable_gravity && enable_g_terms
    figure(3)
    hold on
    plot(T2,X2(:,5))
    plot(T2,X2(:,6))
    plot(T2,X2(:,7))
    plot(T2,X2(:,8))
    plot(T2,X2(:,9))
    hold off;
    legend('p_1','p_2','p_3','p_4','p_5')
    grid on; title('Parameter Estimates Over Time')
end

e = X2(:,1:2) - [qd1(:,1) qd2(:,1)];
edot = X2(:,3:4) - [qd1(:,2) qd2(:,2)];

figure(4)
subplot 211
hold on
plot(T2,e(:,1))
plot(T2,e(:,2))
hold off;
legend('e_1','e_2')
grid on; title('Position Error Output Over Time')
subplot 212
hold on;
plot(T2,edot(:,1))
plot(T2,edot(:,2))
hold off;
legend('edot_1','edot_2')
grid on; title('Rate Error Output Over Time')

for i = 1:length(Tvec)
    v = [qd1(i,2); qd2(i,2)] - lambda*e(i,:)';
    a = [qd1(i,3); qd2(i,3)] - lambda*edot(i,:)';
    r = edot(i,:)' + lambda*e(i,:)';
    
    if enable_gravity && enable_g_terms
        Y = [a(1) a(1)+a(2) cos(X2(i,2))*(2*a(1)+a(2))-sin(X2(i,2))*(X2(i,4)*v(1)+X2(i,3)*v(2)+X2(i,4)*v(2)) cos(X2(i,1)) cos(X2(i,1)+X2(i,2));
             0 a(1)+a(2) cos(X2(i,2))*a(1)+sin(X2(i,2))*X2(i,3)*v(1) 0 cos(X2(i,1)+X2(i,2))];
    else
        Y = [a(1) a(1)+a(2) cos(X2(i,2))*(2*a(1)+a(2))-sin(X2(i,2))*(X2(i,4)*v(1)+X2(i,3)*v(2)+X2(i,4)*v(2));
             0 a(1)+a(2) cos(X2(i,2))*a(1)+sin(X2(i,2))*X2(i,3)*v(1)];
    end
    
    if enable_adaptive && enable_gravity && enable_g_terms
        pest = X2(i,5:9)';
    elseif enable_adaptive
        pest = X2(i,5:7)';
    elseif enable_robust
        pest = globalP;
    end
    u = Y*pest-K*r;
    
    for j = 1:length(u)
        if (u(j) < 0 && abs(u(j)) > max_torque)
            u(j) = -max_torque;
        elseif (u(j) > 0 && abs(u(j)) > max_torque)
            u(j) = max_torque;
        end
    end
    
    U(i,:) = u';
end

figure(5) 
hold on
plot(T2,U(:,1))
plot(T2,U(:,2))
hold off;
legend('\tau_1','\tau_2')
grid on; title('Control Torque Input Over Time')
