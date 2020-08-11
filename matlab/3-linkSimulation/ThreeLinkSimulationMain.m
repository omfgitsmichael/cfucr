%% 3-Link Robotic Arm Dynamics and Control %%
close all; clear all; clc;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

%% Time for Simulations %%
T0 = 0; Tf = 25; Delt = 0.01;
Tvec = linspace(T0,Tf,Tf/Delt);

%% Declare Global Variables %%
global enable_gravity enable_g_terms enable_robust enable_adaptive enable_filter;
global lambda K gamma max_torque rho epsilon alpha filter_order meas_noise n_links;

% Enable certain fields %
enable_gravity = true;
enable_g_terms = true;
enable_filter = true;

% Can only use robust or adaptive, cannot use both when using passivity %
enable_robust = false;
enable_adaptive = true;

% Currently not used except for generating noise %
gyro_noise = 0.005; % deg/sec/sqrt(Hz)
sampling_rate = 1000;
gyro_n = gyro_noise*sqrt(sampling_rate); % deg/sec
gyro_n = gyro_n * pi/180;
meas_noise = [0.035; gyro_n];
n_links = 3;

% Linear Control Gains - Must be Diagonal Matrices %
% Amplifys Noise too much, need these to be smaller than ideal simulation %
lambda = [3.5355 0 0;
          0 3.5355 0;
          0 0 3.5355]*0.75;
K = [7.071 0 0;
     0 7.071 0;
     0 0 7.071]*0.75;

% Maximum allowable torque from motors %
max_torque = [50; 25; 9.13];

% Rate of Adaptivity
if enable_g_terms
    gamma = [2 0 0 0 0 0 0 0 0;
             0 2 0 0 0 0 0 0 0;
             0 0 2 0 0 0 0 0 0;
             0 0 0 2 0 0 0 0 0;
             0 0 0 0 2 0 0 0 0;
             0 0 0 0 0 2 0 0 0;
             0 0 0 0 0 0 10 0 0;
             0 0 0 0 0 0 0 10 0;
             0 0 0 0 0 0 0 0 10];
         
    if enable_gravity
        gamma = [2 0 0 0 0 0 0 0 0;
                 0 2 0 0 0 0 0 0 0;
                 0 0 2 0 0 0 0 0 0;
                 0 0 0 2 0 0 0 0 0;
                 0 0 0 0 2 0 0 0 0;
                 0 0 0 0 0 2 0 0 0;
                 0 0 0 0 0 0 0.01 0 0;
                 0 0 0 0 0 0 0 0.01 0;
                 0 0 0 0 0 0 0 0 0.01];
    end
else
    gamma = [2 0 0 0 0 0; 
             0 2 0 0 0 0;
             0 0 2 0 0 0;
             0 0 0 2 0 0;
             0 0 0 0 2 0;
             0 0 0 0 0 2];
end

% Maximum unmodeled dynamics to be considered as uncertainty %
if enable_robust
    rho = 1.0;
    epsilon = 0.001;
end

if enable_filter
    alpha = [0.85 0.72 0.70;
             0.83 0.75 0.70];
    filter_order = 3;
end

%% Nominal Parameters %%
m1 = 2; % Mass of link 1 (kg) %
lc1 = 0.03; % Length to CG of link 1 (meters) %
l1 = 0.3; % Length of link 1 (meters) %

I1 = (1/3)*m1*l1^2;

m2 = 2; % Mass of link 2 (kg) %
lc2 = 0.03; % Length to CG of link 2 (meters) %
l2 = 0.3; % Length of link 2 (meters) %

I2 = (1/3)*m2*l2^2;

m3 = 2; % Mass of link 3 (kg) %
lc3 = 0.03; % Length to CG of link 3 (meters) %
l3 = 0.3; % Length of link 3 (meters) %

I3 = (1/3)*m3*l3^2;

%% True Parameters %%
p1true = m1*lc1^2+m2*l1^2+m3*l1^2+I1;
p2true = m2*lc2^2+m3*l2^2+I2;
p3true = m3*lc3^2+I3;
p4true = m2*l1*lc2+m3*l1*l2;
p5true = m3*l1*lc3;
p6true = m3*l2*lc3;

pt = [p1true; p2true; p3true; p4true; p5true; p6true];

if enable_gravity
    p7true = (m1*lc1+m2*l1+m3*l1)*9.81;
    p8true = (m2*lc2+m3*l2)*9.81;
    p9true = m3*lc3*9.81;
    
    pt = [p1true; p2true; p3true; p4true; p5true; p6true; p7true; p8true; p9true];
end

%% Initial parameters and estimates %%
pi1(1,1) = 0.4;
pi1(2,1) = 0.3;
pi1(3,1) = 0.2;
pi1(4,1) = 0.35;
pi1(5,1) = 0.25;
pi1(6,1) = 0.15;

if enable_g_terms
    pi1(7,1) = 0.0;
    pi1(8,1) = 0.0;
    pi1(9,1) = 0.0;
    
    if enable_gravity
        pi1(7,1) = 12.0;
        pi1(8,1) = 8.0;
        pi1(9,1) = 4.0;
    end
end
    
th1 = 0;
th2 = 0;
th3 = 0;
thd1 = 0;
thd2 = 0;
thd3 = 0;

x1 = [th1; th2; th3; thd1; thd2; thd3];

%% Initialize Parameters %%
p = pi1;

if enable_filter
    for i = 1:filter_order
        q_prev(:,i) = zeros(n_links,1);
        qdot_prev(:,i) = zeros(n_links,1);
    end
end

flag1 = true;
flag2 = true;
%% Main Simulation Loop %%
for i = 1:length(Tvec)
    tic
    % Receive angle measurements %
    q(1,1) = x1(1)+meas_noise(1)*randn(1);
    q(2,1) = x1(2)+meas_noise(1)*randn(1);
    q(3,1) = x1(3)+meas_noise(1)*randn(1);
    
    % Receive angular rate measurements %
    qdot(1,1) = x1(4)+meas_noise(2)*randn(1);
    qdot(2,1) = x1(5)+meas_noise(2)*randn(1);
    qdot(3,1) = x1(6)+meas_noise(2)*randn(1);
    
    % save the measured parameters %
    qMeas(i,:) = [q' qdot'];
    
    if enable_filter
        for j = 1:n_links
            for k = 1:filter_order
                % Filter the measurements if desired %
                [q(j,1), q_prev(j,k)] = low_pass_filter(q(j),q_prev(j,k),alpha(1,k));
                [qdot(j,1), qdot_prev(j,k)] = low_pass_filter(qdot(j),qdot_prev(j,k),alpha(2,k));
            end
        end

        % save the filtered parameters %
        q_saved(:,i) = q';
        qdot_saved(:,i) = qdot';
    end
    
    % Calculate the Desired Parameters %
    qd(1,1) = acos(0.45-0.45*sin(5*Tvec(i)+pi/2));
    qd(2,1) = -qd(1,1);
    qd(3,1) = qd(1,1);

    qd_des(1,1) = -2.25*sin(5*Tvec(i))./sqrt(1-0.2025*(1-cos(5*Tvec(i))).^2);
    qd_des(2,1) = -qd_des(1,1);
    qd_des(3,1) = qd_des(1,1);

    qdd_des(1,1) = -11.25*cos(5*Tvec(i))./sqrt(1-0.2025*(1-cos(5*Tvec(i))).^2)-...
        (2.27813*(sin(5*Tvec(i)).^2).*(1-cos(5*Tvec(i))))/...
        (1-0.2025*(1-cos(5*Tvec(i))).^2).^(3/2);
    qdd_des(2,1) = -qdd_des(1,1);
    qdd_des(3,1) = qdd_des(1,1);

    qd(1,1) = qd(1,1)/4;
    qd(2,1) = qd(2,1)/2;
    qd(3,1) = qd(3,1)/2;

    qd_des(1,1) = qd_des(1,1)/4;
    qd_des(2,1) = qd_des(2,1)/2;
    qd_des(3,1) = qd_des(3,1)/2;

    qdd_des(1,1) = qdd_des(1,1)/4;
    qdd_des(2,1) = qdd_des(2,1)/2;
    qdd_des(3,1) = qdd_des(3,1)/2;
    
    % Save off the desired trajectory %
    qd1(i,:) = [qd(1,1) qd_des(1,1) qdd_des(1,1)];
    qd2(i,:) = [qd(2,1) qd_des(2,1) qdd_des(2,1)];
    qd3(i,:) = [qd(3,1) qd_des(3,1) qdd_des(3,1)];
    
    % Calculate the Errors %
    e = q - qd;
    edot = qdot - qd_des;
    
    % Save off the rest of the Instrumentation Data %
    X(i,:) = x1';
    P(i,:) = p';
    E(i,:) = [e' edot'];
    
    % Calculate Passivity-Based Linear Control Variables %
    v = qd_des - lambda*e;
    a = qdd_des - lambda*edot;
    r = edot + lambda*e;
    
    % Calculate the Regressor Matrix %
    if enable_g_terms
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
    
    % Calculate the Control %
    if enable_robust && ~enable_adaptive
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
        
        if enable_robust
            if norm(Y'*r) > epsilon
                delp = -rho*Y'*r/norm(Y'*r);
            else
                delp = -(rho/epsilon)*Y'*r;
            end
        end
        % Define the Control %
        u = Y*p-K*r;
        
        if enable_robust
            u = Y*(p+delp)-K*r;
        end
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
    toc
    
    % Update the true parameters if desired
    lc2 = 0.1*cos(Tvec(i))+0.2; % Length to CG of link 3 (meters) %
    l2 = 0.2*cos(Tvec(i))+0.4; % Length of link 3 (meters) %
    lc3 = 0.1*cos(Tvec(i))+0.2; % Length to CG of link 3 (meters) %
    l3 = 0.2*cos(Tvec(i))+0.4; % Length of link 3 (meters) %

    I3 = (1/3)*m3*l3^2;

    p1true = m1*lc1^2+m2*l1^2+m3*l1^2+I1;
    p2true = m2*lc2^2+m3*l2^2+I2;
    p3true = m3*lc3^2+I3;
    p4true = m2*l1*lc2+m3*l1*l2;
    p5true = m3*l1*lc3;
    p6true = m3*l2*lc3;

    pt = [p1true; p2true; p3true; p4true; p5true; p6true];

    if enable_gravity
        p7true = (m1*lc1+m2*l1+m3*l1)*9.81;
        p8true = (m2*lc2+m3*l2)*9.81;
        p9true = m3*lc3*9.81;

        pt = [p1true; p2true; p3true; p4true; p5true; p6true; p7true; p8true; p9true];
    end
    
    % Run the Simulations Continuous Dynamics %
    x0 = x1;
    t = [Tvec(i); Tvec(i) + Delt];
    [t2,X1] = ode45(@(t,x)ThreeLinkDynamics(t,x,pt,u),t,x0,options);
    x1 = X1(end,:)';
end

%% Plot the Instrumented Data %%
figure(1)
subplot 211
hold on
plot(Tvec, X(:,1))
plot(Tvec, X(:,2))
plot(Tvec, X(:,3))
plot(Tvec, qd1(:,1))
plot(Tvec, qd2(:,1))
plot(Tvec, qd3(:,1))
hold off;
legend('\theta_1','\theta_2','\theta_3','\theta_1_{des}','\theta_2_{des}','\theta_3_{des}')
grid on; title('Controlled \theta Output Over Time')
subplot 212
hold on;
plot(Tvec, X(:,4))
plot(Tvec, X(:,5))
plot(Tvec, X(:,6))
plot(Tvec, qd1(:,2))
plot(Tvec, qd2(:,2))
plot(Tvec, qd3(:,2))
hold off;
legend('\omega_1','\omega_2','\omega_3','\omega_1_{des}','\omega_2_{des}','\omega_3_{des}')
grid on; title('Controlled \omega Output Over Time')

figure(2)
hold on
plot(Tvec, P(:,1))
plot(Tvec, P(:,2))
plot(Tvec, P(:,3))
plot(Tvec, P(:,4))
plot(Tvec, P(:,5))
plot(Tvec, P(:,6))
hold off;
legend('p_1','p_2','p_3','p_4','p_5','p_6')
grid on; title('Parameter Estimates Over Time')

if enable_g_terms
    figure(2)
    hold on
    plot(Tvec, P(:,1))
    plot(Tvec, P(:,2))
    plot(Tvec, P(:,3))
    plot(Tvec, P(:,4))
    plot(Tvec, P(:,5))
    plot(Tvec, P(:,6))
    plot(Tvec, P(:,7))
    plot(Tvec, P(:,8))
    plot(Tvec, P(:,9))
    hold off;
    legend('p_1','p_2','p_3','p_4','p_5','p_6','p_7','p_8','p_9')
    grid on; title('Parameter Estimates Over Time')
end

figure(3)
subplot 211
hold on
plot(Tvec, E(:,1))
plot(Tvec, E(:,2))
plot(Tvec, E(:,3))
hold off;
legend('e_1','e_2','e_3')
grid on; title('Position Error Output Over Time')
subplot 212
hold on;
plot(Tvec, E(:,4))
plot(Tvec, E(:,5))
plot(Tvec, E(:,6))
hold off;
legend('edot_1','edot_2','edot_3')
grid on; title('Rate Error Output Over Time')

figure(4) 
hold on
plot(Tvec, U(:,1))
plot(Tvec, U(:,2))
plot(Tvec, U(:,3))
hold off;
legend('\tau_1','\tau_2','\tau_3')
grid on; title('Control Torque Input Over Time')

for i = 1:n_links
    if enable_filter
        figure(4+i)
        subplot 211
        hold on
        plot(Tvec, q_saved(i,:))
        plot(Tvec, qMeas(:,i))
        if i == 1
            plot(Tvec, qd1(:,1))
        elseif i == 2
            plot(Tvec, qd2(:,1))
        elseif i == 3
            plot(Tvec, qd3(:,1))
        end
        hold off;
        legend('\theta_1_{filtered}','\theta_1','\theta_1_{des}')
        grid on; title('filtered \theta Output Over Time')
        subplot 212
        hold on;
        plot(Tvec, qdot_saved(i,:))
        plot(Tvec, qMeas(:,3+i))
        if i == 1
            plot(Tvec, qd1(:,2))
        elseif i == 2
            plot(Tvec, qd2(:,2))
        elseif i == 3
            plot(Tvec, qd3(:,2))
        end
        hold off;
        legend('\omega_1_{filtered}','\omega_1','\omega_1_{des}')
        grid on; title('filtered \omega Output Over Time')
    end
end