function dx = TwoLinkDynamics(t,x,pt,u)
t
global enable_gravity;

dx = zeros(4,1); 

p1 = pt(1);
p2 = pt(2);
p3 = pt(3);
p4 = pt(4);
p5 = pt(5);

q = x(1:2);
qdot = x(3:4);

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

if enable_gravity    
    G(1,1) = p4*cos(q(1))+p5*cos(q(1)+q(2));
    G(2,1) = p5*cos(q(1)+q(2));
    
    dx(3:4) = pinv(M)*(u-C*qdot-G);
else
    dx(3:4) = pinv(M)*(u-C*qdot);
end

M^-1
end