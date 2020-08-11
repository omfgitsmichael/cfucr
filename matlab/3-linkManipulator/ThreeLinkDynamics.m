function dx = ThreeLinkDynamics(t,x,pt)
t
global enable_gravity;

dx = zeros(6,1); 

p1 = pt(1);
p2 = pt(2);
p3 = pt(3);
p4 = pt(4);
p5 = pt(5);
p6 = pt(6);
p7 = pt(7);
p8 = pt(8);
p9 = pt(9);

q = x(1:3);
qdot = x(4:6);

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

if enable_gravity    
    G(1,1) = p7*cos(q(1))+p8*cos(q(1)+q(2))+p9*cos(q(1)+q(2)+q(3));
    G(2,1) = p8*cos(q(1)+q(2))+p9*cos(q(1)+q(2)+q(3));
    G(3,1) = p9*cos(q(1)+q(2)+q(3));
    
    dx(4:6) = M^-1*(C*qdot-G);
else
    dx(4:6) = M^-1*(C*qdot);
end

end