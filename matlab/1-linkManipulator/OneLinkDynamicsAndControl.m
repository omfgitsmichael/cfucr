function dx = OneLinkDynamicsAndControl...
    (t,x,pt,T,qd1,control_type)
t
global enable_gravity enable_g_terms enable_robust enable_adaptive;
global lambda K gamma max_torque rho epsilon globalP;

if enable_adaptive && enable_gravity && enable_g_terms
    dx = zeros(4,1);
elseif enable_adaptive
    dx = zeros(3,1);
else
    dx = zeros(2,1);
end    

q = x(1);
qdot = x(2);

if enable_adaptive && enable_gravity && enable_g_terms
    p = x(3:4);
elseif enable_adaptive
    p = x(3);
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

if enable_gravity
    p2 = ptrue(2);
end

q_des(1,1) = interp1(T,qd1(:,1),t);
qd_des(1,1) = interp1(T,qd1(:,2),t);
qdd_des(1,1) = interp1(T,qd1(:,3),t);

e = q - q_des;
ed = qdot - qd_des;
if control_type == 'passivity'
    
    v = qd_des - lambda*e;
    a = qdd_des - lambda*ed;
    r = ed + lambda*e;

    % passivity-based regressor %
    if enable_gravity && enable_g_terms
        Y = [a(1) cos(q(1))];
    else
        Y = a(1);
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

M11 = p1;

M = M11;

C11 = 0;

C = C11;
 
dx(1) = qdot;

if enable_adaptive && enable_gravity && enable_g_terms
    dx(3:4) = -pinv(gamma)*Y'*r;
elseif enable_adaptive
    dx(3) = -pinv(gamma)*Y'*r;
end

if enable_gravity    
    G(1,1) = p2*cos(q(1));
    
    dx(2) = pinv(M)*(u-C*qdot-G);
else
    dx(2) = pinv(M)*(u-C*qdot);
end

end