u = [1 1 1 1 1 2 2 2 2 2 3 3 3 3 3 4 4 4 4 4 5 5 5 5 5]

xd    = u(1:3);
b1d   = u(4:6);

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = reshape(u(22:24),3,1);
t     = u(end);


R_d = eye(3);
Omega_d = [0; 0.1; 1];
J = diag([P.Jxx; P.Jyy; P.Jzz]);

e_R = vee(R_d'*R - R'*R_d)*0.5;

e_omega = Omega - R'*R_d*Omega_d;
e_R_3= [0; 0; e_R(3)];


f = (P.kx*( x(3)-xd(3)) + P.mass*P.gravity )/(dot(e_R_3,R*e_R_3));
M = e_R*(-P.kR) + e_omega*(-P.kOmega) + cross(Omega,J*Omega);

out = [f;M;];
