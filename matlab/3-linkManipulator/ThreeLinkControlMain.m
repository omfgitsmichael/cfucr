%% 3-Link Robotic Arm Dynamics and Control %%
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
lambda = [3.5355 0 0;
          0 3.5355 0;
          0 0 3.5355]*2.25;
K = [7.071 0 0;
     0 7.071 0;
     0 0 7.071]*2.25;

% Maximum allowable torque from motors %
max_torque = [50; 25; 10];

% Rate of Adaptivity
if enable_g_terms
    gamma = [5 0 0 0 0 0 0 0 0;
             0 5 0 0 0 0 0 0 0;
             0 0 5 0 0 0 0 0 0;
             0 0 0 5 0 0 0 0 0;
             0 0 0 0 5 0 0 0 0;
             0 0 0 0 0 5 0 0 0;
             0 0 0 0 0 0 0.02 0 0;
             0 0 0 0 0 0 0 0.02 0;
             0 0 0 0 0 0 0 0 0.02];
else
    gamma = [5 0 0 0 0 0; 
             0 5 0 0 0 0;
             0 0 5 0 0 0;
             0 0 0 5 0 0;
             0 0 0 0 5 0;
             0 0 0 0 0 5];
end

% Maximum unmodeled dynamics to be considered as uncertainty %
if enable_robust && ~enable_adaptive
    rho = 5.0;
    epsilon = 0.001;
end

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

if enable_gravity
    p7 = 12.0;
    p8 = 8.0;
    p9 = 4.0;
end

if enable_robust && ~enable_adaptive
    globalP = [p1; p2; p3; p4; p5; p6];
    
    if enable_gravity
        globalP = [p1; p2; p3; p4; p5; p6; p7; p8; p9];
    end
end
    
th1 = qdes1(1)+(5*pi/180);
th2 = qdes2(1)+(5*pi/180);
th3 = qdes3(1)+(5*pi/180);
thd1 = qdot_des1(1)+(1*pi/180);
thd2 = qdot_des2(1)+(1*pi/180);
thd3 = qdot_des3(1)+(1*pi/180);

if enable_adaptive
    x02 = [th1; th2; th3; thd1; thd2; thd3; p1; p2; p3; p4; p5; p6];

    if enable_gravity && enable_g_terms
        x02 = [th1; th2; th3; thd1; thd2; thd3; p1; p2; p3; p4; p5; p6; p7; p8; p9];
    end
end

if enable_robust
    x02 = [th1; th2; th3; thd1; thd2; thd3];
end

%% Nominal Parameters %%
m1 = 2; % Mass of link 1 (kg) %
m2 = 2; % Mass of link 2 (kg) %
m3 = 2; % Mass of link 3 (kg) %
lc1 = 0.15; % Length of link 1 (meters) %
l1 = 0.3; % Length of link 1 (meters) %
lc2 = 0.15; % Length of link 2 (meters) %
l2 = 0.3; % Length of link 2 (meters) %
lc3 = 0.15; % Length of link 3 (meters) %
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

qd1 = [qdes1' qdot_des1' qddot_des1'];
qd2 = [qdes2' qdot_des2' qddot_des2'];
qd3 = [qdes3' qdot_des3' qddot_des3'];

%% Robotic Arm Control %%
[T2,X2] = ode45(@(t,x)ThreeLinkDynamicsAndControl(t,x,p,Tvec,qd1,qd2,qd3,'passivity')...
    ,Tvec,x02,options);

%% Graphs and Simulations %%
figure(1)
subplot 211
hold on
plot(T2,X2(:,1))
plot(T2,X2(:,2))
plot(T2,X2(:,3))
plot(T2, qd1(:,1))
plot(T2, qd2(:,1))
plot(T2, qd3(:,1))
hold off;
legend('\theta_1','\theta_2','\theta_3','\theta_1_{des}','\theta_2_{des}','\theta_3_{des}')
grid on; title('Controlled \theta Output Over Time')
subplot 212
hold on;
plot(T2,X2(:,4))
plot(T2,X2(:,5))
plot(T2,X2(:,6))
plot(T2, qd1(:,2))
plot(T2, qd2(:,2))
plot(T2, qd3(:,2))
hold off;
legend('\omega_1','\omega_2','\omega_3','\omega_1_{des}','\omega_2_{des}','\omega_3_{des}')
grid on; title('Controlled \omega Output Over Time')

figure(2)
hold on
plot(T2,X2(:,7))
plot(T2,X2(:,8))
plot(T2,X2(:,9))
plot(T2,X2(:,10))
plot(T2,X2(:,11))
plot(T2,X2(:,12))
hold off;
legend('p_1','p_2','p_3','p_4','p_5','p_6')
grid on; title('Parameter Estimates Over Time')

if enable_gravity && enable_g_terms
    figure(2)
    hold on
    plot(T2,X2(:,7))
    plot(T2,X2(:,8))
    plot(T2,X2(:,9))
    plot(T2,X2(:,10))
    plot(T2,X2(:,11))
    plot(T2,X2(:,12))
    plot(T2,X2(:,13))
    plot(T2,X2(:,14))
    plot(T2,X2(:,15))
    hold off;
    legend('p_1','p_2','p_3','p_4','p_5','p_6','p_7','p_8','p_9')
    grid on; title('Parameter Estimates Over Time')
end

e = X2(:,1:3) - [qd1(:,1) qd2(:,1) qd3(:,1)];
edot = X2(:,4:6) - [qd1(:,2) qd2(:,2) qd3(:,2)];

figure(3)
subplot 211
hold on
plot(T2,e(:,1))
plot(T2,e(:,2))
plot(T2,e(:,3))
hold off;
legend('e_1','e_2','e_3')
grid on; title('Position Error Output Over Time')
subplot 212
hold on;
plot(T2,edot(:,1))
plot(T2,edot(:,2))
plot(T2,edot(:,3))
hold off;
legend('edot_1','edot_2', 'edot_3')
grid on; title('Rate Error Output Over Time')

for i = 1:length(Tvec)
    v = [qd1(i,2); qd2(i,2); qd3(i,2)] - lambda*e(i,:)';
    a = [qd1(i,3); qd2(i,3); qd3(i,3)] - lambda*edot(i,:)';
    r = edot(i,:)' + lambda*e(i,:)';
    
    q = X2(i,1:3)';
    qdot = X2(i,4:6)';
    
    if enable_gravity && enable_g_terms
        Y(1,1) = a(1);
        Y(1,2) = a(1)+a(2);
        Y(1,3) = a(1)+a(2)+a(3);
        Y(1,4) = cos(q(2))*(2*a(1)+a(2))...
            -sin(q(2))*(qdot(2)*v(1)+(qdot(1)+qdot(2))*v(2));
        Y(1,5) = cos(q(2)+q(3))*(2*a(1)+a(2)+a(3))...
            -sin(q(2)+q(3))*((qdot(2)+qdot(3))*v(1)...
            +(qdot(1)+qdot(2)+qdot(3))*v(2)+(qdot(1)+qdot(2)+qdot(3))*v(3));
        Y(1,6) = cos(q(3))*(2*a(1)+2*a(2)+a(3))...
            -sin(q(3))*(qdot(3)*v(1)+qdot(3)*v(2)+(qdot(1)+qdot(2)+qdot(3))*v(3));
        Y(1,7) = cos(q(1));
        Y(1,8) = cos(q(1)+q(2));
        Y(1,9) = cos(q(1)+q(2)+q(3));
        
        Y(2,1) = 0;
        Y(2,2) = a(1)+a(2);
        Y(2,3) = a(1)+a(2)+a(3);
        Y(2,4) = cos(q(2))*a(1)+sin(q(2))*qdot(1)*v(1);
        Y(2,5) = cos(q(2)+q(3))*a(1)+sin(q(2)+q(3))*qdot(1)*v(1);
        Y(2,6) = cos(q(3))*(2*a(1)+2*a(2)+a(3))...
            -sin(q(3))*(qdot(3)*v(1)+qdot(3)*v(2)+(qdot(1)+qdot(2)+qdot(3))*v(3));
        Y(2,7) = 0;
        Y(2,8) = cos(q(1)+q(2));
        Y(2,9) = cos(q(1)+q(2)+q(3));
        
        Y(3,1) = 0;
        Y(3,2) = 0;
        Y(3,3) = a(1)+a(2)+a(3);
        Y(3,4) = 0;
        Y(3,5) = cos(q(2)+q(3))*a(1)+sin(q(2)+q(3))*qdot(1)*v(1);
        Y(3,6) = cos(q(3))*(a(1)+a(2))+sin(q(3))*(qdot(1)+qdot(2))*(v(1)+v(2));
        Y(3,7) = 0;
        Y(3,8) = 0;
        Y(3,9) = cos(q(1)+q(2)+q(3));
    else
        Y(1,1) = a(1);
        Y(1,2) = a(1)+a(2);
        Y(1,3) = a(1)+a(2)+a(3);
        Y(1,4) = cos(q(2))*(2*a(1)+a(2))...
            -sin(q(2))*(qdot(2)*v(1)+(qdot(1)+qdot(2))*v(2));
        Y(1,5) = cos(q(2)+q(3))*(2*a(1)+a(2)+a(3))...
            -sin(q(2)+q(3))*((qdot(2)+qdot(3))*v(1)...
            +(qdot(1)+qdot(2)+qdot(3))*v(2)+(qdot(1)+qdot(2)+qdot(3))*v(3));
        Y(1,6) = cos(q(3))*(2*a(1)+2*a(2)+a(3))...
            -sin(q(3))*(qdot(3)*v(1)+qdot(3)*v(2)+(qdot(1)+qdot(2)+qdot(3))*v(3));
        
        Y(2,1) = 0;
        Y(2,2) = a(1)+a(2);
        Y(2,3) = a(1)+a(2)+a(3);
        Y(2,4) = cos(q(2))*a(1)+sin(q(2))*qdot(1)*v(1);
        Y(2,5) = cos(q(2)+q(3))*a(1)+sin(q(2)+q(3))*qdot(1)*v(1);
        Y(2,6) = cos(q(3))*(2*a(1)+2*a(2)+a(3))...
            -sin(q(3))*(qdot(3)*v(1)+qdot(3)*v(2)+(qdot(1)+qdot(2)+qdot(3))*v(3));
        
        Y(3,1) = 0;
        Y(3,2) = 0;
        Y(3,3) = a(1)+a(2)+a(3);
        Y(3,4) = 0;
        Y(3,5) = cos(q(2)+q(3))*a(1)+sin(q(2)+q(3))*qdot(1)*v(1);
        Y(3,6) = cos(q(3))*(a(1)+a(2))+sin(q(3))*(qdot(1)+qdot(2))*(v(1)+v(2));
    end
    
    if enable_adaptive && enable_gravity && enable_g_terms
        pest = X2(i,7:15)';
    elseif enable_adaptive
        pest = X2(i,7:12)';
    elseif enable_robust
        pest = globalP;
    end
    u = Y*pest-K*r;
    
    for j = 1:length(u)
        if (u(j) < 0 && abs(u(j)) > max_torque(j))
            u(j) = -max_torque(j);
        elseif (u(j) > 0 && abs(u(j)) > max_torque(j))
            u(j) = max_torque(j);
        end
    end
    
    U(i,:) = u';
end

figure(4) 
hold on
plot(T2,U(:,1))
plot(T2,U(:,2))
plot(T2,U(:,3))
hold off;
legend('\tau_1','\tau_2','\tau_3')
grid on; title('Control Torque Input Over Time')
