function dx = TwoLinkDynamics(t,x,m1,m2,lc1,l1,lc2,I1,I2,enable_gravity)

dx = zeros(4,1);

q = x(1:2);
qdot = x(3:4);

M11 = m1*lc1^2+m2*l1^2+2*m2*l1*lc2*cos(q(2))+m2*lc2^2+I1+I2;
M12 = m2*l1*lc2*cos(q(2))+m2*lc2^2+I2;
M21 = m2*l1*lc2*cos(q(2))+m2*lc2^2+I2;
M22 = m2*lc2^2+I2;

M = [M11 M12;
     M21 M22];

C11 = -m2*l1*lc2*sin(q(2))*qdot(2);
C12 = -m2*l1*lc2*sin(q(2))*qdot(1)-m2*l1*lc2*sin(q(2))*qdot(2);
C21 = m2*l1*lc2*sin(q(2))*qdot(1);
C22 = 0;

C = [C11 C12;
     C21 C22];

G = zeros(2,1);

if (enable_gravity == true)
    g = 9.81;
    
    G(1) = (m1*lc1+m2*l1)*g*cos(q(1))+m2*lc2*g*cos(q(1)+q(2));
    G(2) = m2*lc2*g*cos(q(1)+q(2));
end

dx(1:2) = qdot;

dx(3:4) = pinv(M)*(-C*qdot-G);

end