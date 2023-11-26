function m = R(phi,theta,psi)
m=Rz(phi)*Ry(theta)*Rx(psi);
% m=Rx(psi)*Ry(theta)*Rz(phi);
end

function mx = Rx(psi)
psi = psi*pi/180;
e11 = 1; e12 = 0;        e13 = 0;
e21 = 0; e22 = cos(psi); e23 = -sin(psi);
e31 = 0; e32 = sin(psi); e33 = cos(psi);
mx = [e11,e12,e13;e21,e22,e23;e31,e32,e33];
end
function my = Ry(theta)
theta = theta*pi/180;
e11 = cos(theta); e12 = 0; e13 = sin(theta);
e21 = 0;          e22 = 1; e23 = 0;
e31 = -sin(theta);e32 = 0; e33 = cos(theta);
my = [e11,e12,e13;e21,e22,e23;e31,e32,e33];
end
function mz = Rz(phi)
phi = phi*pi/180;
e11 = cos(phi); e12 = -sin(phi); e13 = 0;
e21 = sin(phi); e22 = cos(phi);  e23 = 0;
e31 = 0;        e32 = 0;         e33 = 1;
mz = [e11,e12,e13;e21,e22,e23;e31,e32,e33];
end
