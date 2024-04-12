fs=10;
dt=1/fs;
IMU = imuSensor('accel-gyro','SampleRate',fs);
g=9.81;
acc_specs=accelparams('MeasurementRange', 15*g, ...
    'NoiseDensity',40e-6*g*100,'RandomWalk',23/1000/60/dt); % *100 more noisy than the specs
% walk; 60 is because sqrt(hr) = 60 sqrt(s)
gyro_specs=gyroparams('MeasurementRange',deg2rad(430) ...
    ,'NoiseDensity', deg2rad(0.3)/3600 *1000, 'RandomWalk',deg2rad(0.005)/60/dt);
% *1000 more noisy than the specs
% conversion from velocity random walk to acceleration random walk may not be
% correct, same with gyro, current conversion is /dt .
IMU.Accelerometer=acc_specs;
IMU.Gyroscope=gyro_specs;
%% Defining ground truth and IMU output
rng(10); % control the random number (seed)
runtime=1000;
N=runtime*fs;
t = (0:(N-1))/IMU.SampleRate;
accBody=zeros(N,3);
angVelBody=zeros(N,3);
% Define acceleration and angular velocity
firstLoopNumSamples = N*0.4;
secondLoopNumSamples = N*0.6;
totalNumSamples = firstLoopNumSamples + secondLoopNumSamples;
accBody(1:firstLoopNumSamples,1)=1;
accBody(firstLoopNumSamples+1:end,1)=1;
accBody(1:firstLoopNumSamples,2)=1;
accBody(firstLoopNumSamples+1:end,2)=-1;
% angVelBody(:,3) = sin(0.01*t);
% angVelBody(:,2) = cos(0.01*t);
angVelBody(1:firstLoopNumSamples,1)=0;
angVelBody(firstLoopNumSamples+1:end,2)=0;
[accReading,gyroReading]=IMU(accBody,angVelBody);
%IMU adds g to z direction and flip all the signs of inputs for no good reasons
accReading(:,3)=accReading(:,3)-g; 
accReading=-accReading;
traj = kinematicTrajectory('SampleRate',fs);
[pos_GT,quat_GT,vel_GT,~,~] = traj(accBody,angVelBody);
%% March time

N=length(accBody);
quat=ones(N,1,'quaternion');
pos=zeros(N,3);
vel=zeros(N,3);
acc=zeros(N,3);
% initialize the system state. When attitude is quaternion(1,0,0,0),
% the global acc is the local acc.
quat(1)=quaternion(1,0,0,0);
pos(1,:)=[0,0,0];
vel(1,:)=[0,0,0];
acc(1,:)=accReading(1,:);
for i=1:N-1
    [quat(i+1),pos(i+1,:),vel(i+1,:),acc(i,:)]=March_time(quat(i),vel(i,:),pos(i,:),accBody(i,:),angVelBody(i,:),dt);
end

%% Raw
traj1 = kinematicTrajectory('SampleRate',fs);
[pos_raw,quat_raw,vel_raw,~,~] = traj1(accReading,gyroReading);
% [pos_raw,quat_raw,vel_raw,~,~] = traj1(accBody,angVelBody);
%% KF
kf=KF();
% for unknown reasons the std reading from IMU about 0.705 of the set noise density
kf.sg2 = power(IMU.Gyroscope.NoiseDensity(1)*sqrt(fs)*0.705,2);
kf.sa2 = power(IMU.Accelerometer.NoiseDensity(1)*sqrt(fs)*0.705,2);
kf.ab = IMU.Accelerometer.RandomWalk(1);
kf.wb = IMU.Gyroscope.RandomWalk(1);
kf.g=[0;0;0];
kf.x=zeros(10,1);
kf.x(7)=1;
kf.P = zeros(9,9);
% kf.P(1:3,1:3) = eye(3)*kf.sg2;
quat_KF=zeros(N,4);
pos=zeros(N,3);
vel=zeros(N,3);
err_p=zeros(N,3);
err_v=zeros(N,3);
err_q=zeros(N,3);
z_gps=zeros(N,7); % first 4 is quaternion, last 3 is position
z_gps(1,1) = 1;
R_gps = ones(6,1);
R_gps(1:3) = power(0.5,2); % error in gps position
R_gps(4:6) = power(0.1/180*pi,2); % error in gps attitude 0.1 degrees
R_gps = diag(R_gps);
for i=1:N-1
    quat_KF(i,:) = kf.q;
    pos(i,:) = kf.p;
    vel(i,:) = kf.v;
    err_p(i,:) = sqrt(diag(kf.P(1:3,1:3)));
    err_v(i,:) = sqrt(diag(kf.P(4:6,4:6)));
    err_q(i,:) = sqrt(diag(kf.P(7:9,7:9)));
%     z=[angVelBody(i,:),accBody(i,:)].';
    z = [gyroReading(i,:),accReading(i,:)].';
%     z = [angVelBody(i,:),accReading(i,:)].';
    % z = [gyroReading(i,:),accBody(i,:)].';
    kf.propagate(z,dt);
    % gps signal is fused with predicted position and attitude

    z_gps(i+1,1:3) = pos_GT(i+1,:) + transpose(sqrt(R_gps(1:3,1:3))*randn(3,1));
    z_gps(i+1,4:7) = compact(quatmultiply(quat_GT(i+1,:),...
        exp(quaternion([0,randn(1,3)/2*sqrt(R_gps(4:6,4:6))]))));
%     z_gps(i+1,1:3) = compact(quat_GT(i+1,:));
%     z_gps(i+1,4:7) = pos_GT(i+1,:);
    kf.update(z_gps(i+1,:), R_gps);
end
quat_KF(end,:) = kf.q;
quat_KF = quaternion(quat_KF);
pos(end,:) = kf.p;
vel(end,:) = kf.v;
err_v(end,:) = sqrt(diag(kf.P(7:9,7:9)));
err_p(end,:) = sqrt(diag(kf.P(4:6,4:6)));
%%
rms(accReading)/sqrt(kf.sa2)
std(gyroReading)/sqrt(kf.sg2)

%%
% quat2eul(quat_GT(end-10:end,:))/pi*180
quat2eul(quaternion(z_gps(1:10,4:7)))/pi*180
eul_GPS(1:10,:)
%%
% rms(eul_raw)
% rms(eul_GPS)
% rms(eul_KF)
% std(quat2eul(quaternion(z_gps(:,4:7))))
% std(eul_GPS)
std(eul_KF)
%%
IMU.Gyroscope.RandomWalk(1)
%%
pos(1:10,:)
pos_raw(1:10,:)
%%
quat2eul(quaternion([1,err_q(9999,:)/2]))/pi*180
std(eul_KF)
%%
std(pos-pos_GT)
mean(err_p)
%% Check if std converted by noise density equal to std
% no, the former is 2/3 of the later.
temp=IMU.Accelerometer.NoiseDensity*sqrt(fs)
% temp(3)
std(accReading(:,3))
% mean(acc(:,3))/2*10000
% pos(end,3)
%%
index=2;
f=figure(2);

f.Position=[600 300 1600 900];

% upper = vel+err_v;
% lower = vel-err_v;
% 
% upper = pos+err_p;
% lower = pos-err_p;

% data = [z_gps(:,1:3), pos_raw, pos, pos_GT] ;
% legends = ["GPS","Raw","KF","Ground Truth"] ;

% data = [z_gps(:,1:3), pos, pos_GT] ;
% legends = ["GPS","KF","Ground Truth"] ;
% 
% YLabel = ["x-position","y-position","z-position"];

% data = [z_gps(:,1:3) - pos_GT, pos - pos_GT];
% legends = ["GPS","KF"];

% upper = vel+err_v;
% lower = vel-err_v;
% hold on;

% YLabel = ["x-velocity","y-velocity","z-velocity"];
% 
% data = [z_gps(:,1:3) - pos_GT, pos - pos_GT];
% legends = ["GPS","KF"];

% hold off
% fill([t, fliplr(t)], [upper(:,index).', fliplr(lower(:,index).')], 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% eul_GT = quat2eul(quat_GT)/pi*180;
% eul_KF = quat2eul(quat_KF)/pi*180;
% % eul_raw = quat2eul(quat_raw)/pi*180;
% eul_GPS = quat2eul(quaternion(z_gps(:,4:7)))/pi*180;
% legends = ["GPS","KF","GT"];
% data = [eul_GPS,eul_KF,eul_GT];

YLabel = ["heading","pitch","roll"];

eul_KF = quat2eul(quatmultiply(quatconj(quat_KF),quat_GT))/pi*180;
eul_raw = quat2eul(quatmultiply(quatconj(quat_raw),quat_GT))/pi*180;
eul_GPS = quat2eul(quatmultiply(quatconj(quaternion(z_gps(:,4:7))),quat_GT))/pi*180;
data = [eul_raw,eul_GPS, eul_KF];
legends = ["raw","GPS","KF"];

Plot(t, data, index, legends)
xlabel('t (sec)',"FontSize",16)
ylabel(YLabel(index),"FontSize",16)
legend("FontSize",18)
ax = gca;
ax.FontSize = 14;
grid on;
% xlim([0,100])
%% funct plot
function Plot(t, data, index, legends)
[~,col_num] = size(data);
data_num = round(col_num/3);
hold on;
for i=1:data_num
    plot(t, data(:,index),'DisplayName',legends(i))
    index = index + 3;
end
hold off;
end

%% funct march time
function [q,r,v,a]=March_time(q0,v0,r0,accBody,w,dt)
w_norm=norm(w);
if norm(w/w_norm) > 1+eps & w_norm ~= 0
    disp('angular velocity is too small');
    return;
end
if w_norm~=0
    integrator0=quaternion(cos(w_norm/2*dt),w(1)/w_norm*sin(w_norm/2*dt),w(2)/w_norm*sin(w_norm/2*dt),w(3)/w_norm*sin(w_norm/2*dt));
else
    integrator0=quaternion(1,0,0,0);
end
q=quatmultiply(integrator0,q0);
R0=quat2rotm(q0);
a=R0*accBody.';
a=a.';
v=v0+a*dt;
r=r0+v0*dt+1/2*a*dt*dt;
end
