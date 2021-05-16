function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
xd    = u(1:3);
b1d   = u(4:6);

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = reshape(u(22:24),3,1);
t     = u(end);


R_d = [1,0,0;0,1,0;0,0,1];
Omega_d = [0; 0.1; 1];
J = diag([P.Jxx; P.Jyy; P.Jzz]);
eR = 0.5*vee((R_d)'*R - (R)'*R_d);
eOmega = Omega - R'*R_d*Omega_d;
e_3= [0; 0; 1];

f = (P.kx*( x(3)-xd(3))+P.kx*v(3) + P.mass*P.gravity )/(dot(e_3,R*e_3));
M = eR*(-P.kR) + eOmega*(-P.kOmega) + cross(Omega,J*Omega);

out = [f;M;eR];
end