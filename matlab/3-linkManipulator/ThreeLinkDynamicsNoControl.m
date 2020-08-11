%% 3-Link Robotic Arm Dynamics and Control %%
close all; clear all; clc;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

%% Time for Simulations %%
T0 = 0; Tf = 10; Delt = 0.01;
Tvec = linspace(T0,Tf,Tf/Delt);

%% Declare Global Variables %%
global enable_gravity enable_g_terms;

% Enable certain fields %
enable_gravity = true;
enable_g_terms = true;

%% Desired Path %%
qdes1 = acos(0.45-0.45*sin(5*Tvec+pi/2));
qdes2 = -qdes1;
qdes3 = qdes1;

qdot_des1 = -2.25*sin(5*Tvec)./sqrt(1-0.2025*(1-cos(5*Tvec)).^2);
qdot_des2 = -qdot_des1;
qdot_des3 = qdot_des1;

qddot_des1 = -11.25*cos(5*Tvec)./sqrt(1-0.2025*(1-cos(5*Tvec)).^2)-...
    (2.27813*(sin(5*Tvec).^2).*(1-cos(5*Tvec)))/...
    (1-0.2025*(1-cos(5*Tvec)).^2).^(3/2);
qddot_des2 = -qddot_des1;
qddot_des3 = qddot_des1;

qdes1 = qdes1/2;
qdes2 = qdes2;
qdes3 = qdes3;

qdot_des1 = qdot_des1/2;
qdot_des2 = qdot_des2;
qdot_des3 = qdot_des3;

qddot_des1 = qddot_des1/2;
qddot_des2 = qddot_des2;
qddot_des3 = qddot_des3;

%% Initial parameters and estimates %%
p1 = 0.3;
p2 = 0.2;
p3 = 0.1;
p4 = 0.3;
p5 = 0.2;
p6 = 0.1;
    
th1 = qdes1(1)+(5*pi/180);
th2 = qdes2(1)+(5*pi/180);
th3 = qdes3(1)+(5*pi/180);
thd1 = qdot_des1(1)+(1*pi/180);
thd2 = qdot_des2(1)+(1*pi/180);
thd3 = qdot_des3(1)+(1*pi/180);

x02 = [th1; th2; th3; thd1; thd2; thd3];

%% Nominal Parameters %%
m1 = 2; % Mass of link 1 (kg) %
m2 = 2; % Mass of link 2 (kg) %
m3 = 2; % Mass of link 3 (kg) %
lc1 = 0.27; % Length of link 1 (meters) %
l1 = 0.3; % Length of link 1 (meters) %
lc2 = 0.27; % Length of link 2 (meters) %
l2 = 0.3; % Length of link 2 (meters) %
lc3 = 0.27; % Length of link 3 (meters) %
l3 = 0.3; % Length of link 3 (meters) %

I1 = (1/3)*m1*l1^2;
I2 = (1/3)*m2*l2^2;
I3 = (1/3)*m3*l3^2;

%% True Parameters %%
p1true = m1*lc1^2+m2*l1^2+m3*l1^2+I1;
p2true = m2*lc2^2+m3*l2^2+I2;
p3true = m3*lc3^2+I3;
p4true = m2*l1*lc2+m3*l1*l2;
p5true = m3*l1*lc3;
p6true = m3*l2*lc3;

p = [p1true; p2true; p3true; p4true; p5true; p6true];

if enable_gravity
    p7true = (m1*lc1+m2*l1+m3*l1)*9.81;
    p8true = (m2*lc2+m3*l2)*9.81;
    p9true = m3*lc3*9.81;
    
    p = [p1true; p2true; p3true; p4true; p5true; p6true; p7true; p8true; p9true];
end

%% Robotic Arm Control %%
[T2,X2] = ode45(@(t,x)ThreeLinkDynamics(t,x,p),Tvec,x02,options);

%% Graphs and Simulations %%
figure(1)
subplot 211
hold on
plot(T2,X2(:,1))
plot(T2,X2(:,2))
plot(T2,X2(:,3))
hold off;
legend('\theta_1','\theta_2','\theta_3')
grid on; title('Controlled \theta Output Over Time')
subplot 212
hold on;
plot(T2,X2(:,4))
plot(T2,X2(:,5))
plot(T2,X2(:,6))
hold off;
legend('\omega_1','\omega_2','\omega_3')
grid on; title('Controlled \omega Output Over Time')
