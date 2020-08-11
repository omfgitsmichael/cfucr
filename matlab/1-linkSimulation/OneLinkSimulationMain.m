%% 1-Link Robotic Arm Dynamics and Control %%
close all; clear all; clc;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

%% Time for Simulations %%
T0 = 0; Tf = 20; Delt = 0.01;
Tvec = linspace(T0,Tf,Tf/Delt);

%% Declare Global Variables %%
global enable_gravity enable_g_terms enable_robust enable_adaptive enable_filter;
global lambda K gamma max_torque rho epsilon alpha filter_order meas_noise;

% Enable certain fields %
enable_gravity = true;
enable_g_terms = true;
enable_filter = true;

% Can only use robust or adaptive, cannot use both %
enable_robust = false;
enable_adaptive = true;

% Currently not used except for generating noise %
meas_noise = [0.02; 0.03];

% Linear Control Gains - Must be Diagonal Matrices %
% Amplifys Noise too much, need these to be smaller than ideal simulation %
lambda = 3.5355*0.75;
K = 7.071*0.75;

% Maximum allowable torque from motors %
max_torque = 75.0;

% Rate of Adaptivity
if enable_g_terms
    gamma = [2 0; 0 0.01];
else
    gamma = 2;
end

% Maximum unmodeled dynamics to be considered as uncertainty %
if enable_robust && ~enable_adaptive
    rho = 10.0;
    epsilon = 0.001;
end

if enable_filter
    alpha = [0.88 0.75 0.75;
             0.92 0.90 0.88];
    filter_order = 3;
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

%% Nominal Parameters %%
m1 = 2; % Mass of link 1 (kg) %
lc1 = 0.27; % Length of link 1 (meters) %
l1 = 0.3; % Length of link 1 (meters) %

I1 = (1/3)*m1*l1^2;

%% True Parameters %%
p1true = m1*lc1^2+I1;

pt = p1true;

if enable_gravity
    p2true = m1*lc1*9.81;
    
    pt = [p1true; p2true];
end

%% Initial parameters and estimates %%
pi1(1,1) = 0.05;

if enable_gravity && enable_g_terms
    pi1(2,1) = 2.0;
end
    
th1 = qdes1(1)+(5*pi/180);
thd1 = qdot_des1(1)+(2*pi/180);

x1 = [th1; thd1];

qd1 = [qdes1' qdot_des1' qddot_des1'];

%% Initialize Parameters %%
p = pi1;

if enable_filter
    for i = 1:filter_order
        q_prev(i) = 0;
        qdot_prev(i) = 0;
    end
end

flag1 = true;
flag2 = true;
%% Main Simulation Loop %%
for i = 1:length(Tvec)
    % Receive angle measurements %
    q = x1(1)+meas_noise(1)*randn(1);
    
    % Receive angular rate measurements %
    qdot = x1(2)+meas_noise(2)*randn(1);
    
    % save the measured parameters %
    qMeas(i,:) = [q' qdot'];
    
    if enable_filter
        for j = 1:filter_order
            % Filter the measurements if desired %
            [q, q_prev(j)] = low_pass_filter(q,q_prev(j),alpha(1,j));
            [qdot, qdot_prev(j)] = low_pass_filter(qdot,qdot_prev(j),alpha(2,j));
        end

        % save the filtered parameters %
        q_saved(i) = q;
        qdot_saved(i) = qdot;
    end
    
    % Calculate the Desired Parameters %
    qd = qd1(i,1);
    qd_des = qd1(i,2);
    qdd_des = qd1(i,3);
    
    % Calculate the Errors %
    e = q - qd;
    edot = qdot - qd_des;
    
    % Save off the Instrumentation Data %
    X(i,:) = x1';
    P(i,:) = p';
    E(i,:) = [e' edot'];
    
    % Calculate Passivity-Based Linear Control Variables %
    v = qd_des - lambda*e;
    a = qdd_des - lambda*edot;
    r = edot + lambda*e;
    
    % Calculate the Regressor Matrix %
    if enable_gravity && enable_g_terms
        Y = [a(1) cos(q(1))];
    else
        Y = a(1);
    end
    
    % Calculate the Control %
    if enable_robust
        if norm(Y'*r) > epsilon
            delp = -rho*Y'*r/norm(Y'*r);
        else
            delp = -(rho/epsilon)*Y'*r;
        end
        
        % Define the Control %
        u = Y*(p+delp)-K*r;
        
    elseif enable_adaptive
        % Update Parameter Estimates %
        F = -pinv(gamma)*Y'*r;
        p = p + F*Delt;
        
        % Define the Control %
        u = Y*p-K*r;
    end
    
    % Limit the Torque Input %
    for j = 1:length(u)
        if (u(j) < 0 && abs(u(j)) > max_torque(j))
            u(j) = -max_torque(j);
        elseif (u(j) > 0 && abs(u(j)) > max_torque(j))
            u(j) = max_torque(j);
        end
    end
    
    % Instrument the Control Torque %
    U(i,:) = u';
    
    % Update the true parameters if desired
%     if (Tvec(i) > 5) && flag1 == true
%         pt = pt*2;
%         flag1 = false;
%     elseif (Tvec(i) > 10) && flag2 == true
%         pt = pt/4;
%         flag2 = false;
%     end
    
    % Run the Simulations Continuous Dynamics %
    x0 = x1;
    t = [Tvec(i); Tvec(i) + Delt];
    [t2,X1] = ode45(@(t,x)OneLinkDynamics(t,x,pt,u),t,x0,options);
    x1 = X1(end,:)';
end

%% Plot the Instrumented Data %%
figure(1)
subplot 211
hold on
plot(Tvec, X(:,1))
plot(Tvec, qd1(:,1))
hold off;
legend('\theta_1','\theta_1_{des}')
grid on; title('Controlled \theta Output Over Time')
subplot 212
hold on;
plot(Tvec, X(:,2))
plot(Tvec, qd1(:,2))
hold off;
legend('\omega_1','\omega_1_{des}')
grid on; title('Controlled \omega Output Over Time')

figure(2)
hold on
plot(Tvec, P(:,1))
hold off;
legend('p_1')
grid on; title('Parameter Estimates Over Time')

if enable_gravity && enable_g_terms
    figure(2)
    hold on
    plot(Tvec, P(:,1))
    plot(Tvec, P(:,2))
    hold off;
    legend('p_1','p_2')
    grid on; title('Parameter Estimates Over Time')
end

figure(3)
subplot 211
hold on
plot(Tvec, E(:,1))
hold off;
legend('e_1')
grid on; title('Position Error Output Over Time')
subplot 212
hold on;
plot(Tvec, E(:,2))
hold off;
legend('edot_1')
grid on; title('Rate Error Output Over Time')

figure(4) 
hold on
plot(Tvec, U(:,1))
hold off;
legend('\tau_1')
grid on; title('Control Torque Input Over Time')

if enable_filter
    figure(5)
    subplot 211
    hold on
    plot(Tvec, q_saved)
    plot(Tvec, qMeas(:,1))
    plot(Tvec, qd1(:,1))
    hold off;
    legend('\theta_1_{filtered}','\theta_1','\theta_1_{des}')
    grid on; title('filtered \theta Output Over Time')
    subplot 212
    hold on;
    plot(Tvec, qdot_saved)
    plot(Tvec, qMeas(:,2))
    plot(Tvec, qd1(:,2))
    hold off;
    legend('\omega_1_{filtered}','\omega_1','\omega_1_{des}')
    grid on; title('filtered \omega Output Over Time')
end