kf=KF();
kf.g=[0;0;9.81];
kf.x=zeros(10,1);
kf.x(1)=1;
kf.P=eye(9)*0.01;

%%
z=[1;0;9.81;pi/2;0;0];
kf.propagate(z,1);
kf.x
kf.P
%%
R_gps = eye(3)*0.5*100
%%
sin(t).'