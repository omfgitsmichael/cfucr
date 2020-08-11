function dx = OneLinkDynamics(t,x,pt,u)
t
global enable_gravity;

dx = zeros(2,1); 

q = x(1);
qdot = x(2);

M11 = pt(1);
M = M11;

C11 = 0;
C = C11;
 
dx(1) = qdot;

if enable_gravity    
    G(1,1) = pt(2)*cos(q(1));
    dx(2) = pinv(M)*(u-C*qdot-G);
else
    dx(2) = pinv(M)*(u-C*qdot);
end
pinv(M)
end