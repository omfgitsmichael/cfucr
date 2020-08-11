function dx = ThreeLinkDynamicsAndControl...
    (t,x,pt,T,qd1,qd2,qd3,control_type)
t
global enable_gravity enable_g_terms enable_robust enable_adaptive;
global lambda K gamma max_torque rho epsilon globalP;

if enable_adaptive && enable_gravity && enable_g_terms
    dx = zeros(15,1);
elseif enable_adaptive
    dx = zeros(12,1);
else
    dx = zeros(6,1);
end    

q = x(1:3);
qdot = x(4:6);

if enable_adaptive && enable_gravity && enable_g_terms
    p = x(7:15);
elseif enable_adaptive
    p = x(7:12);
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
p4 = ptrue(4);
p5 = ptrue(5);
p6 = ptrue(6);

if enable_gravity
    p7 = ptrue(7);
    p8 = ptrue(8);
    p9 = ptrue(9);
end

q_des(1,1) = interp1(T,qd1(:,1),t);
q_des(2,1) = interp1(T,qd2(:,1),t);
q_des(3,1) = interp1(T,qd3(:,1),t);

qd_des(1,1) = interp1(T,qd1(:,2),t);
qd_des(2,1) = interp1(T,qd2(:,2),t);
qd_des(3,1) = interp1(T,qd3(:,2),t);

qdd_des(1,1) = interp1(T,qd1(:,3),t);
qdd_des(2,1) = interp1(T,qd2(:,3),t);
qdd_des(3,1) = interp1(T,qd3(:,3),t);

e = q - q_des;
ed = qdot - qd_des;
if control_type == 'passivity'
    
    v = qd_des - lambda*e;
    a = qdd_des - lambda*ed;
    r = ed + lambda*e;

    % passivity-based regressor %
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
        if (u(i) < 0 && abs(u(i)) > max_torque(i))
            u(i) = -max_torque(i);
        elseif (u(i) > 0 && abs(u(i)) > max_torque(i))
            u(i) = max_torque(i);
        end
    end
        
elseif control_type == 'active'
    u = 0;
end

M11 = p1+p2+p3+2*p4*cos(q(2))+2*p5*cos(q(2)+q(3))+2*p6*cos(q(3));
M12 = p2+p3+p4*cos(q(2))+p5*cos(q(2)+q(3))+2*p6*cos(q(3));
M13 = p3+p5*cos(q(2)+q(3))+p6*cos(q(3));
M21 = p2+p3+p4*cos(q(2))+p5*cos(q(2)+q(3))+2*p6*cos(q(3));
M22 = p2+p3+2*p6*cos(q(3));
M23 = p3+p6*cos(q(3));
M31 = p3+p5*cos(q(2)+q(3))+p6*cos(q(3));
M32 = p3+p6*cos(q(3));
M33 = p3;

M = [M11 M12 M13;
     M21 M22 M23;
     M31 M32 M33];

C11 = -p4*sin(q(2))*qdot(2)-p5*sin(q(2)+q(3))*(qdot(2)+qdot(3))-p6*sin(q(3))*qdot(3);
C12 = -p4*sin(q(2))*(qdot(1)+qdot(2))-p5*sin(q(2)+q(3))*(qdot(1)+qdot(2)+qdot(3))-p6*sin(q(3))*qdot(3);
C13 = -p5*sin(q(2)+q(3))*(qdot(1)+qdot(2)+qdot(3))-p6*sin(q(3))*(qdot(1)+qdot(2)+qdot(3));
C21 = -p6*sin(q(3))*qdot(3)+p4*sin(q(2))*qdot(1)+p5*sin(q(2)+q(3))*qdot(1);
C22 = -p6*sin(q(3))*qdot(3);
C23 = -p6*sin(q(3))*(qdot(1)+qdot(2)+qdot(3));
C31 = p5*sin(q(2)+q(3))*qdot(1)+p6*sin(q(3))*(qdot(1)+qdot(2));
C32 = p6*sin(q(3))*(qdot(1)+qdot(2));
C33 = 0;

C = [C11 C12 C13;
     C21 C22 C23;
     C31 C32 C33];
 
dx(1:3) = qdot;

if enable_adaptive && enable_gravity && enable_g_terms
    dx(7:15) = -pinv(gamma)*Y'*r;
elseif enable_adaptive
    dx(7:12) = -pinv(gamma)*Y'*r;
end

if enable_gravity    
    G(1,1) = p7*cos(q(1))+p8*cos(q(1)+q(2))+p9*cos(q(1)+q(2)+q(3));
    G(2,1) = p8*cos(q(1)+q(2))+p9*cos(q(1)+q(2)+q(3));
    G(3,1) = p9*cos(q(1)+q(2)+q(3));
    
    dx(4:6) = pinv(M)*(u-C*qdot-G);
else
    dx(4:6) = pinv(M)*(u-C*qdot);
end

end