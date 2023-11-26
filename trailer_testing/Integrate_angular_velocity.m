[t_raw,boreas_Gxyz]=Read_Boreas_raw("23Jan18/jan18_afternoon_raw_sensors.txt");
[t_sys,boreas_hpr]=Read_Boreas_system_state("23Jan18/jan18_afternoon_system_state.txt");
ti=ones(1,1);
t_raw=Change_t_to_datetime(t_raw);
t_raw=Change_t_to_seconds(t_raw,ti);
t_sys=Change_t_to_datetime(t_sys);
t_sys=Change_t_to_seconds(t_sys,ti);
% 
%%
boreas_Gxyz(:,2)=-boreas_Gxyz(:,2);
%% Read CPT7
[t,cpt7_Gxyz,cpt7_hpr]=Read_CPT7("23March28/cpt7_afternoon.txt");
%% Time interval
num_of_interval=8;
ti=zeros(num_of_interval*2,1);
% CPT7 mode: Land:
% ti(1)=194523;ti(2)=194843;
% ti(3)=194959;ti(4)=195405;
% ti(5)=195505;ti(6)=195827;
% ti(7)=195941;ti(8)=200339;
% ti(9)=200500;ti(10)=200814;
% ti(11)=200933;ti(12)=201335;
% ti(13)=201448;ti(14)=201755;
% ti(15)=201910;ti(16)=202309;
% CPT7 mode: Foot:
ti(1)=202547;ti(2)=202856;
ti(3)=202959;ti(4)=203401;
ti(5)=203513;ti(6)=203818;
ti(7)=203939;ti(8)=204337;
ti(9)=204457;ti(10)=204805;
ti(11)=204921;ti(12)=205320;
ti(13)=205437;ti(14)=205748;
ti(15)=205905;ti(16)=210330;
ti(17)=210335;ti(18)=211946;
ti=Set_time_interval(t_utc,ti,num_of_interval); % change ti to be the index
t=Change_t_to_datetime(t_utc); 
t=Change_t_to_seconds(t,ti);
%% Integrate to get attitude
t1=3600; 
t2=t1+1000; % from t1 to t2 the offset is calculated.
% attitude=get_attitude(boreas_Gxyz,boreas_hpr,t_raw,t1,t2);
%% Quaternion integration
R0 = R(boreas_hpr(t1,1),boreas_hpr(t1,2),boreas_hpr(t1,3));
% quat_array=zeros([length(t_raw),1],'quaternion');
[V,D]=eig(R0);
k=V(:,3);
% check if eigenvalue of rotation matrix is 1
if isreal(D(3,3)) ~= 1
    disp("eigenvalue not real");
end
theta=acos((trace(R0)-1)/2);
% quat=quaternion(cos(theta/2),-k(1)*sin(theta/2),-k(2)*sin(theta/2),-k(3)*sin(theta/2));
% quat_array(t1)=quat;
quat=quaternion(rotm2quat(R0));
roll_offset=-mean(boreas_Gxyz(t1:t2,1));
pitch_offset=-mean(boreas_Gxyz(t1:t2,2));
heading_offset=-mean(boreas_Gxyz(t1:t2,3));
attitude=zeros(length(t_raw),3);
attitude(t1,:)=get_angle(R0)/180*pi;
for i=t1+1:length(t_raw)
    dt=t_raw(i)-t_raw(i-1);
    w=boreas_Gxyz(i,:);
    w(1)=w(1)+roll_offset;
    w(2)=w(2)+pitch_offset;
    w(3)=w(3)+heading_offset;
    w=w/180*pi;
    r = w(1)*dt;
    p = w(2)*dt;
    h = w(3)*dt;
    theta=acos((trace(R(h,p,r))-1)/2);
    % when w is too small, dividing by it may rise a problem.
    if norm(w/norm(w)) > 1+eps
        disp('w is too small');
        disp(i);
        break
    end
    integrator0=quaternion(cos(norm(w)/2*dt),w(1)/norm(w)*sin(norm(w)/2*dt),w(2)/norm(w)*sin(norm(w)/2*dt),w(3)/norm(w)*sin(norm(w)/2*dt));
%     integrator0=quaternion(cos(theta/2),w(1)/norm(w)*sin(theta/2),w(2)/norm(w)*sin(theta/2),w(3)/norm(w)*sin(theta/2));
    quat=quatmultiply(integrator0,quat);
    % V is eigenvector and D is a diagonal matrix of eigenvalues
    
    [q0,q1,q2,q3]=parts(quat);
    h=atan2(2 * ((q1 * q2) + (q0 * q3)),q0^2 + q1^2 - q2^2 - q3^2);
    p=asin(2 * ((q0 * q2)-(q1 * q3)));
    r=atan2(2*((q2*q3) + (q0 * q1)),q0^2 - q1^2 - q2^2 + q3^2);
    
%     [h,p,r]=quat2angle(quat);

%     using rotation matrices to march time
%     r = w(1)*dt;
%     p = w(2)*dt;
%     h = w(3)*dt;
%     R0=R(h,p,r)*R0;
%     hpr=get_angle(R0);
%     attitude(i,:)=hpr;
    attitude(i,:)=[h,p,r];
    % test if quat2angle is equivalent to the conversion by hand.
%     if abs(atan2(2 * ((q1 * q2) + (q0 * q3)),q0^2 + q1^2 - q2^2 - q3^2)-h) > 4*eps ...
%         | abs(asin(2 * ((q0 * q2)-(q1 * q3)))-p) > 4*eps ...
%         | abs(atan2(2*((q2*q3) + (q0 * q1)),q0^2 - q1^2 - q2^2 + q3^2)-r) > 4*eps
%         disp(i);
%         break
%     end
end
attitude(:,:)=attitude(:,:)/pi*180;
attitude(:,1)=attitude(:,1)+360;

%% Plot attitude
Title=["heading","pitch","roll"];
hpr_index=3;
f=figure(1);
f.Position=[500 200 1600 900];
plot(t_raw,attitude(:,hpr_index),'--');
hold on;
plot(t_sys,boreas_hpr(:,hpr_index));
hold off;
legend("Integration","Boreas","FontSize",18);
xlim([t_raw(t1),t_raw(end)]);
% ylim([245.5,246]);
xlabel('Time (s)',"FontSize",11);
ylabel('Degree',"FontSize",11);
title(Title(hpr_index));
grid on;
ax = gca;
ax.FontSize = 12;

%% Plot histogram
hpr_index=2;
Title=["heading","pitch","roll"];
f=figure(4);
f.Position=[500 200 800 450];
Xlim=[];
for i=1:3
    subplot(3,1,i);
    Plot_histogram(attitude(3600:end,i),Title(i),Xlim,"degree",Title(i));
end

%% function Integrate angular velocity to get attitude
function attitude=get_attitude(boreas_Gxyz,boreas_hpr,t_raw,t1,t2)

% t_raw=t_raw-t1+1; t_sys=t_sys-t1+1;
% frequency=10; t1=t1*frequency;
roll_offset=-mean(boreas_Gxyz(t1:t2,1))
pitch_offset=-mean(boreas_Gxyz(t1:t2,2))
heading_offset=-mean(boreas_Gxyz(t1:t2,3))
attitude=zeros(length(t_raw),3);
R0 = R(boreas_hpr(t1,1),boreas_hpr(t1,2),boreas_hpr(t1,3));
attitude(t1,:)=get_angle(R0);
for i=t1+1:length(t_raw)
    dt=t_raw(i)-t_raw(i-1);
    temp1 = boreas_Gxyz(i-1,1)*dt+roll_offset*dt;
    temp2 = boreas_Gxyz(i-1,2)*dt+pitch_offset*dt;
    temp3 = boreas_Gxyz(i-1,3)*dt+heading_offset*dt;
%     temp1 = boreas_Gxyz(i-1,1)*dt;
%     temp2 = boreas_Gxyz(i-1,2)*dt;
%     temp3 = boreas_Gxyz(i-1,3)*dt;
    attitude(i,:)=get_angle(R(temp3,temp2,temp1)*R0);
    R0=R(temp3,temp2,temp1)*R0;
end
attitude(:,1)=attitude(:,1)+360;
end
%% function plot histogram
function []=Plot_histogram(y1,Title,Xlim,Xlabel,Ylabel)
histfit(y1);
pd = fitdist(y1,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title(Title);
xlabel(Xlabel,"FontSize",11);
ylabel(Ylabel,"FontSize",11);
if isempty(Xlim)==0
    xlim(Xlim);
end
ax = gca;
ax.FontSize = 12;
grid on;
end
%% funct Change t to seconds; input t has to be datetime
function t=Change_t_to_seconds(t,ti);
temp=zeros(length(t),1);
for i=1:length(t)
    a=diff([t(ti(1)),t(i)]);
    temp(i)=seconds(a);
end
t=temp;
end
%% funct Change t to datetime; input t has to be double
function t=Change_t_to_datetime(t)
t=num2str(t,'%.3f');
t=datetime(t,"Format",'HHmmss.SSS');
end

%%
%% Funct Read CPT7
function [file_t,file_Gxyz,file_hpr]=Read_CPT7(file_name)
file = readtable(file_name,'Delimiter',{',','*',';'});
N=height(file);
file1_t=zeros(N,1);
file1_Gxyz=zeros(N,3);
file1_hpr=zeros(N,3);
j=0;
k=0;
for i=1:N
    if string(file{i,1}) == '%RAWIMUSA' & file{i,4}>=0.1 
        j=j+1;
        temp=datetime('1980-01-06 00:00:00') + seconds(file{i,4}*7*3600*24 + file{i,5}-18);
        temp=str2double(string(datetime(temp,'Format','HHmmss.SS')));
        file1_t(j)=temp;
        if class(file{i,10})=="cell"
            file1_Gxyz(j,:)=[str2double(file{i,10}),-file{i,11},file{i,12}]; % in cpt7 manual, y is negative
        else
            file1_Gxyz(j,:)=[file{i,10},-file{i,11},file{i,12}];
        end
    elseif string(file{i,1}) == '%INSATTSA' & file{i,4}>=0.1
        k=k+1;
        file1_hpr(k,:)=[file{i,8},file{i,7},file{i,6}];
    end
end
file_Gxyz=file1_Gxyz(1:j,:)*100*2^(-33);
file_t=file1_t(1:j);
file_hpr=file1_hpr(1:k,:);
end
%% function read raw boreas data
function [t,boreas_Gxyz]=Read_Boreas_raw(boreas_name)
boreas = readtable(boreas_name);
boreas = boreas{:,:};
boreas_time = boreas(:,1);
boreas_Gx = boreas(:,5);
boreas_Gy = boreas(:,6);
boreas_Gz = boreas(:,7);
boreas_Gxyz=[boreas_Gx,boreas_Gy,boreas_Gz];
t=boreas_time;
t=datetime(t, 'ConvertFrom', 'posixtime', 'Format', 'HHmmss.SS');
temp=zeros(length(t),1);
for i=1:length(t)
    temp(i)=str2num(string(t(i)));
end
t=temp;
end
%% funct read boreas system state
function [t,boreas_hpr]=Read_Boreas_system_state(boreas_name)
boreas = readtable(boreas_name);
boreas = boreas{:,:};
boreas_time = boreas(:,1);
boreas_h = boreas(:,5);
boreas_p = boreas(:,6);
boreas_r = boreas(:,7);
boreas_hpr=[boreas_h,boreas_p,boreas_r];
t=boreas_time;
t=datetime(t, 'ConvertFrom', 'posixtime', 'Format', 'HHmmss.SS');
temp=zeros(length(t),1);
for i=1:length(t)
    temp(i)=str2num(string(t(i)));
end
t=temp;
end