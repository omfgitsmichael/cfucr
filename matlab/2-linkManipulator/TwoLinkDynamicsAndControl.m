function dx = TwoLinkDynamicsAndControl...
    (t,x,pt,T,qd1,qd2,control_type)
t
global enable_gravity enable_g_terms enable_robust enable_adaptive;
global lambda K gamma max_torque rho epsilon globalP;

if enable_adaptive && enable_gravity && enable_g_terms
    dx = zeros(9,1);
elseif enable_adaptive
    dx = zeros(7,1);
else
    dx = zeros(4,1);
end    

q = x(1:2);
qdot = x(3:4);

if enable_adaptive && enable_gravity && enable_g_terms
    p = x(5:9);
elseif enable_adaptive
    p = x(5:7);
elseif enable_robust
    p = globalP;
end

% Modify true parameters to show adaptivity is possible %
if t > 5
    ptrue = pt/2;
else
    ptrue = pt;
end

p1 = ptrue(1);
p2 = ptrue(2);
p3 = ptrue(3);

if enable_gravity
    p4 = ptrue(4);
    p5 = ptrue(5);
end

q_des(1,1) = interp1(T,qd1(:,1),t);
q_des(2,1) = interp1(T,qd2(:,1),t);

qd_des(1,1) = interp1(T,qd1(:,2),t);
qd_des(2,1) = interp1(T,qd2(:,2),t);

qdd_des(1,1) = interp1(T,qd1(:,3),t);
qdd_des(2,1) = interp1(T,qd2(:,3),t);

e = q - q_des;
ed = qdot - qd_des;
if control_type == 'passivity'
    
    v = qd_des - lambda*e;
    a = qdd_des - lambda*ed;
    r = ed + lambda*e;

    % passivity-based regressor %
    if enable_gravity && enable_g_terms
        Y = [a(1) a(1)+a(2) cos(q(2))*(2*a(1)+a(2))-sin(q(2))*(qdot(2)*v(1)+qdot(1)*v(2)+qdot(2)*v(2)) cos(q(1)) cos(q(1)+q(2));
             0 a(1)+a(2) cos(q(2))*a(1)+sin(q(2))*qdot(1)*v(1) 0 cos(q(1)+q(2))];
    else
        Y = [a(1) a(1)+a(2) cos(q(2))*(2*a(1)+a(2))-sin(q(2))*(qdot(2)*v(1)+qdot(1)*v(2)+qdot(2)*v(2));
             0 a(1)+a(2) cos(q(2))*a(1)+sin(q(2))*qdot(1)*v(1)];
    end
    
    if enable_robust
        if norm(Y'*r) > epsilon
            delp = -rho*Y'*r/norm(Y'*r);
        else
            delp = -(rho/epsilon)*Y'*r;
        end
        
        u = Y*(p+delp)-K*r;
        
    elseif enable_adaptive
        u = Y*p-K*r;
    end
    
    % Limit the torque %
    for i = 1:length(u)
        if (u(i) < 0 && abs(u(i)) > max_torque)
            u(i) = -max_torque;
        elseif (u(i) > 0 && abs(u(i)) > max_torque)
            u(i) = max_torque;
        end
    end
        
elseif control_type == 'active'
    u = 0;
end

M11 = p1+p2+2*p3*cos(q(2));
M12 = p3*cos(q(2))+p2;
M21 = p3*cos(q(2))+p2;
M22 = p2;

M = [M11 M12;
     M21 M22];

C11 = -p3*sin(q(2))*qdot(2);
C12 = -p3*sin(q(2))*qdot(1)-p3*sin(q(2))*qdot(2);
C21 = p3*sin(q(2))*qdot(1);
C22 = 0;

C = [C11 C12;
     C21 C22];
 
dx(1:2) = qdot;

if enable_adaptive && enable_gravity && enable_g_terms
    dx(5:9) = -pinv(gamma)*Y'*r;
elseif enable_adaptive
    dx(5:7) = -pinv(gamma)*Y'*r;
end

if enable_gravity    
    G(1,1) = p4*cos(q(1))+p5*cos(q(1)+q(2));
    G(2,1) = p5*cos(q(1)+q(2));
    
    dx(3:4) = pinv(M)*(u-C*qdot-G);
else
    dx(3:4) = pinv(M)*(u-C*qdot);
end

end