%% 1-Link Robotic Arm Dynamics and Control %%
close all; clear all; clc;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

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
lambda = 3.5355*3.5;
K = 7.071*3.5;

% Maximum allowable torque from motors %
max_torque = 8.0;

% Rate of Adaptivity
if enable_g_terms
    gamma = [5 0; 0 0.2];
else
    gamma = 5;
end

% Maximum unmodeled dynamics to be considered as uncertainty %
if enable_robust && ~enable_adaptive
    rho = 5.0;
    epsilon = 0.001;
end

%% Desired Path %%
qdes1 = acos(0.45-0.45*sin(5*Tvec+pi/2));

qdot_des1 = -2.25*sin(5*Tvec)./sqrt(1-0.2025*(1-cos(5*Tvec)).^2);

qddot_des1 = -11.25*cos(5*Tvec)./sqrt(1-0.2025*(1-cos(5*Tvec)).^2)-...
    (2.27813*(sin(5*Tvec).^2).*(1-cos(5*Tvec)))/...
    (1-0.2025*(1-cos(5*Tvec)).^2).^(3/2);

qdes1 = qdes1/4;

qdot_des1 = qdot_des1/4;

qddot_des1 = qddot_des1/4;

%% Initial parameters and estimates %%
p1 = 0.05;

if enable_gravity
    p2 = 2.0;
end

if enable_robust && ~enable_adaptive
    globalP = p1;
    
    if enable_gravity
        globalP = [p1; p2];
    end
end
    
th1 = qdes1(1)+(5*pi/180);
thd1 = qdot_des1(1)+(2*pi/180);

if enable_adaptive
    x02 = [th1; thd1; p1];

    if enable_gravity && enable_g_terms
        x02 = [th1; thd1; p1; p2];
    end
end

if enable_robust
    x02 = [th1; thd1];
end

%% Nominal Parameters %%
m1 = 2; % Mass of link 1 (kg) %
lc1 = 0.15; % Length of link 1 (meters) %
l1 = 0.3; % Length of link 1 (meters) %

I1 = (1/3)*m1*l1^2;

%% True Parameters %%
p1true = m1*lc1^2+I1;

p = p1true;

if enable_gravity
    p2true = m1*lc1*9.81;
    
    p = [p1true; p2true];
end

qd1 = [qdes1' qdot_des1' qddot_des1'];

%% Robotic Arm Control %%
[T2,X2] = ode45(@(t,x)OneLinkDynamicsAndControl(t,x,p,Tvec,qd1,'passivity')...
    ,Tvec,x02,options);

%% Graphs and Simulations %%
figure(1)
subplot 211
hold on
plot(T2,X2(:,1))
plot(T2, qd1(:,1))
hold off;
legend('\theta_1','\theta_1_{des}')
grid on; title('Controlled \theta Output Over Time')
subplot 212
hold on;
plot(T2,X2(:,2))
plot(T2, qd1(:,2))
hold off;
legend('\omega_1','\omega_1_{des}')
grid on; title('Controlled \omega Output Over Time')

figure(2)
hold on
plot(T2,X2(:,3))
hold off;
legend('p_1')
grid on; title('Parameter Estimates Over Time')

if enable_gravity && enable_g_terms
    figure(2)
    hold on
    plot(T2,X2(:,3))
    plot(T2,X2(:,4))
    hold off;
    legend('p_1','p_2')
    grid on; title('Parameter Estimates Over Time')
end

e = X2(:,1) - qd1(:,1);
edot = X2(:,2) - qd1(:,2);

figure(3)
subplot 211
hold on
plot(T2,e(:,1))
hold off;
legend('e_1')
grid on; title('Position Error Output Over Time')
subplot 212
hold on;
plot(T2,edot(:,1))
hold off;
legend('edot_1')
grid on; title('Rate Error Output Over Time')

for i = 1:length(Tvec)
    v = qd1(i,2) - lambda*e(i,:)';
    a = qd1(i,3) - lambda*edot(i,:)';
    r = edot(i,:)' + lambda*e(i,:)';
    
    if enable_gravity && enable_g_terms
        Y = [a(1) cos(X2(i,1))];
    else
        Y = a(1);
    end
    
    if enable_adaptive && enable_gravity && enable_g_terms
        pest = X2(i,3:4)';
    elseif enable_adaptive
        pest = X2(i,3)';
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

figure(4) 
hold on
plot(T2,U(:,1))
hold off;
legend('\tau_1')
grid on; title('Control Torque Input Over Time')
