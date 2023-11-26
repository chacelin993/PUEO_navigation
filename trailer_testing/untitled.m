R1=R(10,20,30);
R2=R(10,20,25);
get_angle(R1*R2)
[V,D]=eig(R1);
k=V(:,1);
theta=norm(get_angle(R1));
% quat1=quaternion(cos(theta/2),-k(1)*sin(theta/2),-k(2)*sin(theta/2),-k(3)*sin(theta/2));
quat1=quaternion(rotm2quat(R1));
[V,D]=eig(R2);
k=V(:,1);
theta=norm(get_angle(R2));
% quat2=quaternion(cos(theta/2),-k(1)*sin(theta/2),-k(2)*sin(theta/2),-k(3)*sin(theta/2));
quat2=quaternion(rotm2quat(R2));
quat3=quatmultiply(quat1,quat2);
[q0,q1,q2,q3]=parts(quat3);
% [h,p,r]=quat2angle(quat3);
h=atan2(2 * ((q1 * q2) + (q0 * q3)),q0^2 + q1^2 - q2^2 - q3^2);
p=asin(2 * ((q0 * q2)-(q1 * q3)));
r=atan2(2*((q2*q3) + (q0 * q1)),q0^2 - q1^2 - q2^2 + q3^2);
[h,p,r]/pi*180
%%
[V,D]=eig(R1);
k=V(:,1);
theta=norm(get_angle(R1))/180*pi;
[q0,q1,q2,q3]=parts(quaternion(cos(theta/2),-k(1)*sin(theta/2),-k(2)*sin(theta/2),-k(3)*sin(theta/2)));

get_angle(quat2rotm(quaternion(cos(theta/2),-k(1)*sin(theta/2),-k(2)*sin(theta/2),-k(3)*sin(theta/2))))


%%
format long g
[q0,q1,q2,q3]=parts(quaternion(rotm2quat(R1)));
quatmultiply(quatmultiply(quaternion(cos(theta/2),-k(1)*sin(theta/2),-k(2)*sin(theta/2),-k(3)*sin(theta/2)),quaternion(0,k(1),k(2),k(3))), ...
    quaternion(cos(theta/2),k(1)*sin(theta/2),k(2)*sin(theta/2),k(3)*sin(theta/2)))
quatmultiply(quatmultiply(quaternion(rotm2quat(R1)),quaternion(0,k(1),k(2),k(3))),quaternion(q0,-q1,-q2,-q3))
%%
[h,p,r]=quat2angle(quaternion(cos(theta/2),-k(1)*sin(theta/2),-k(2)*sin(theta/2),-k(3)*sin(theta/2)));
[h,p,r]/pi*180
[h,p,r]=quat2angle(quaternion(q0,q1,q2,q3));
[h,p,r]/pi*180
%%
R1=R(10,20,30);
hpr=get_angle(R1)/180*pi;
sqrt(hpr(1)^2+hpr(2)^2+hpr(3)^2)/pi*180
theta=acos((trace(R1)-1)/2)/pi*180
%%
format long g;
R1=R(41,41,41)*R(40,40,40).';
hpr=get_angle(R1)/180*pi;
sqrt(hpr(1)^2+hpr(2)^2+hpr(3)^2)/pi*180
theta=acos((trace(R1)-1)/2)/pi*180
%%
[V,D]=eig(R(1,1,1));
R(45,45,45)
det([sqrt(3)/3,sqrt(3)/3,sqrt(3)/3;-sqrt(3)/3,sqrt(3)/3,-sqrt(3)/3;-sqrt(3)/3,-sqrt(3)/3,sqrt(3)/3])
%%
a=0.5;
R1=R(a,a,a);
(acos((trace(R1)-1)/2)/pi*180-sqrt(a^2*3))/sqrt(a^2*3)
%%
q1=2;q2=1;q3=3;q4=4;
quat1=quaternion(q1,q2,q3,q4);
quat2=quaternion(1,4,5,2);
quatmultiply(quat1,quat2)
[q1,-q2,-q3,-q4;q2,q1,-q4,q3;q3,q4,q1,-q2;q4,-q3,q2,q1]*[1;4;5;2]
%%
P=[1,0;0,2];
error_ellipse(P,[1,1])
