%% IMU settings, calculate ground truth states
fs = 10;
IMU = imuSensor('accel-gyro','SampleRate',fs);
g = 9.81 ;
% acc_specs=accelparams('MeasurementRange', 15*g, ...
%     'NoiseDensity',40e-6*g*100,'RandomWalk',23/1000/60*10); %  noise*100 more noisy than the specs
% % walk; 60 is because sqrt(hr) = 60 sqrt(s)
% gyro_specs=gyroparams('MeasurementRange',deg2rad(430) ...
%     ,'NoiseDensity', deg2rad(0.3)/3600 *1000, 'RandomWalk',deg2rad(0.005)/60*100);

% no measurement range
acc_specs=accelparams('NoiseDensity',40e-6*g*100,'RandomWalk',23/1000/60*10);
gyro_specs=gyroparams('NoiseDensity', deg2rad(0.3)/3600 *1000, 'RandomWalk',deg2rad(0.005)/60*100);

% noise *1000 more noisy than the specs
% conversion from velocity random walk to acceleration random walk may not be
% correct, same with gyro, current conversion is /dt .

% no random walk
% acc_specs=accelparams('MeasurementRange', 15*g, ...
%     'NoiseDensity',40e-6*g*100);
% gyro_specs=gyroparams('MeasurementRange',deg2rad(430) ...
%     ,'NoiseDensity', deg2rad(0.3)/3600 *1000);

% no noise
% acc_specs=accelparams('MeasurementRange', 15*g, ...
%     'NoiseDensity',0,'RandomWalk',23/1000/60*10);
% gyro_specs=gyroparams('MeasurementRange',deg2rad(430) ...
%     ,'NoiseDensity', 0, 'RandomWalk',deg2rad(0.005)/60*100);

IMU.Accelerometer=acc_specs;
IMU.Gyroscope=gyro_specs;
% Defining ground truth and IMU output
rng(10); % control the random number (seed)
runtime=1000;
N=runtime*fs;
t = (0:(N-1))/IMU.SampleRate;
t_GT = t;
% Define acceleration and angular velocity

[accBody,angVelBody] = SetTrueIMUSignal(t, g);
% accBody(:,1)=1;accBody(:,2)=1;accBody(:,3)=1;
%IMU adds g to z direction and flip all the signs of inputs for no good reasons
[accReading,gyroReading]=IMU(accBody,angVelBody);

accReading(:,3)=accReading(:,3)-g; 
accReading=-accReading;
traj = kinematicTrajectory('SampleRate',fs);
[pos_GT,quat_GT,vel_GT,~,~] = traj(accBody,angVelBody);
num_atna = 1;
atna_GT = zeros(N,3,num_atna); % last index indicates antenna number
atna_GT(1,:,1) = [0,0,3];  % defined in global coordinate, right above IMU
atna_GT(1,:,2) = [3,0,0];  % vector from first atna to second, not position.
% atna_GT(1,:,3) = [0,3,0];
% atna_GT(1,:,3) = [-10,0,10];
% atna_GT(1,:,3) = [0,-10,10];
for j = 1:size(atna_GT,3)
    if j==1
        for i = 1:N-1
            atna_GT(i+1,:,j) = pos_GT(i+1,:) + ... 
                (quat2rotm(quat_GT(i+1))*(atna_GT(1,:,j)-pos_GT(1,:)).').';
        end
    else
        for i = 1:N-1
            atna_GT(i+1,:,j) = (quat2rotm(quat_GT(i+1))*atna_GT(1,:,j).').';
        end
    end
end
%%
plot(t_GT, [atna_GT(:,:,2)])
%%
atna_GT(1:10,:,2)
%%
%% Raw
traj1 = kinematicTrajectory('SampleRate',fs);
[pos_raw,quat_raw,vel_raw,~,~] = traj1(accReading,gyroReading);
% [pos_raw,quat_raw,vel_raw,~,~] = traj1(accBody,angVelBody);
%% KF, for GT and KF have same frequency
rng(20);
kf=KF();
% for unknown reasons the std reading from IMU about 0.71 of the set noise density
kf.sa2 = power(IMU.Accelerometer.NoiseDensity(1)*sqrt(fs)*0.71,2);
kf.sg2 = power(IMU.Gyroscope.NoiseDensity(1)*sqrt(fs)*0.71,2);
% kf.sa2 = power(IMU.Accelerometer.NoiseDensity(1)*sqrt(fs),2);
% kf.sg2 = power(IMU.Gyroscope.NoiseDensity(1)*sqrt(fs),2);
% no need to convert to discrete here below, taken care in kf.
kf.saw2 = power(IMU.Accelerometer.RandomWalk(1),2);
kf.sww2 = power(IMU.Gyroscope.RandomWalk(1),2);
kf.g=[0;0;0];
kf.x=zeros(16,1);
kf.x(7)=1;
kf.P = zeros(15,15);
% kf.P(1:3,1:3) =eye(3) ;
quat_KF=zeros(N,4);
pos=zeros(N,3);
vel=zeros(N,3);
err_p=zeros(N,3);
err_v=zeros(N,3);
err_q=zeros(N,3);
z_gps=zeros(N,7); % first 3 is position, last 4 is quaternion
z_gps(1,4) = 1;
vel_GPS = zeros(N,3);
R_gps = ones(6,1);
R_gps(1:3) = power(0.5,2); % error in gps position
R_gps(4:6) = power(0.1/180*pi,2); % error in gps attitude 0.1 degrees
R_gps = diag(R_gps);
bias_a = zeros(N,3);
bias_w = zeros(N,3);
for i=1:N-1
    quat_KF(i,:) = kf.q;
    pos(i,:) = kf.p;
    vel(i,:) = kf.v;
    err_p(i,:) = sqrt(diag(kf.P(1:3,1:3)));
    err_v(i,:) = sqrt(diag(kf.P(4:6,4:6)));
    err_q(i,:) = sqrt(diag(kf.P(7:9,7:9)));
    bias_a(i,:) = kf.ab;
    bias_w (i,:) = kf.wb;
%     z=[angVelBody(i,:),accBody(i,:)].';
    z = [gyroReading(i,:),accReading(i,:)].';
%     z = [angVelBody(i,:),accReading(i,:)].';
%     z = [gyroReading(i,:),accBody(i,:)].';
    kf.propagate(z,dt);
    % gps signal is fused with predicted position and attitude
    z_gps(i+1,1:3) = pos_GT(i+1,:) + transpose(sqrt(R_gps(1:3,1:3))*randn(3,1));
    z_gps(i+1,4:7) = compact(quatmultiply(quat_GT(i+1,:),...
        exp(quaternion([0,randn(1,3)/2*sqrt(R_gps(4:6,4:6))]))));
    kf.update(z_gps(i+1,:), R_gps);
end
quat_KF(end,:) = kf.q;
quat_KF = quaternion(quat_KF);
pos(N,:) = kf.p;
vel(N,:) = kf.v;
err_v(N,:) = sqrt(diag(kf.P(4:6,4:6)));
err_p(N,:) = sqrt(diag(kf.P(1:3,1:3)));
err_q(N,:) = sqrt(diag(kf.P(7:9,7:9)));
bias_a(N,:) = kf.ab;
bias_w (N,:) = kf.wb;
% calculate velocity of GPS measurements
% vel_GPS(:,1) = gradient(z_gps(:,1),t);
% vel_GPS(:,2) = gradient(z_gps(:,2),t);
% vel_GPS(:,3) = gradient(z_gps(:,3),t);

%% KF, calculate errors under different frequencies of IMU
loop_num = 20;
mean_err_p = zeros(loop_num,3); % stores error estimate
mean_err_v = zeros(loop_num,3);
mean_err_q = zeros(loop_num,3);
std_p = zeros(loop_num,3); % stores actual error
std_v = zeros(loop_num,3);
std_q = zeros(loop_num,3);

for j =1:1
disp("begin loop "+ j)
IMU2GPS_fratio = j;
fs=10 * IMU2GPS_fratio;
dt=1/fs;
IMU = imuSensor('accel-gyro','SampleRate',fs);
acc_specs=accelparams('NoiseDensity',40e-6*g*100,'RandomWalk',23/1000/60*10); 
gyro_specs=gyroparams('NoiseDensity', deg2rad(0.3)/3600 *1000, 'RandomWalk',deg2rad(0.005)/60*100);

% gyro_specs=gyroparams('MeasurementRange',deg2rad(430) ...
%     ,'NoiseDensity', deg2rad(0.3)/3600 *1000);
% acc_specs=accelparams('MeasurementRange', 15*g, ...
%     'NoiseDensity',40e-6*g*100);

IMU.Accelerometer=acc_specs;
IMU.Gyroscope=gyro_specs;

% Defining ground truth and IMU output
rng(10); % control the random number (seed)
N=runtime*fs;
t = (0:(N-1))/IMU.SampleRate;
pos_GT_l = interp1(t_GT,pos_GT,t,'linear');
vel_GT_l = interp1(t_GT,vel_GT,t,'linear');
quat_GT_l = Slerp(t_GT,quat_GT,t);
atna_GT_l = interp1(t_GT,atna_GT,t,'linear');
disp("done imu initialzation "+ j)

kf=KF();
% for unknown reasons the reading std from IMU about 0.708 of the set noise density
kf.sa2 = power(IMU.Accelerometer.NoiseDensity(1)*sqrt(fs)*0.708,2);
kf.sg2 = power(IMU.Gyroscope.NoiseDensity(1)*sqrt(fs)*0.708,2);
% no need to convert to discrete here below, taken care in kf.
kf.saw2 = power(IMU.Accelerometer.RandomWalk(1),2);
kf.sww2 = power(IMU.Gyroscope.RandomWalk(1),2);
[accBody,angVelBody] = SetTrueIMUSignal(t,g);

% kf.g=[0;0;0];
kf.g=[0;0;-9.8]; % g in global frame
accBody = accBody - GetLocalg(quat_GT_l, kf.g);

[accReading,gyroReading]=IMU(accBody,angVelBody);
%IMU adds g to z direction and flip all the signs of inputs for no good reasons
accReading(:,3)=accReading(:,3)-g; 
accReading=-accReading;
rng(20);

kf.x=zeros(16+num_atna*3,1);
kf.x(1:3) = pos_GT_l(1,:).'; % initial position
kf.x(4:6) = vel_GT_l(1,:).';
kf.x(7:10)=compact(quat_GT_l(1)).'; % initial attitude
kf.atna_init_vec = zeros(3,1) ;
for i=1:num_atna % Change Later
    if i==1
        kf.atna_init_vec(:,i) = (quat2rotm(quat_GT_l(1)).')*(atna_GT_l(1,:,i)- pos_GT_l(1,:)).';
        kf.x(16+(3*i-2):16+i*3) = atna_GT_l(1,:,i).';
    else
        kf.atna_init_vec(:,i) = (quat2rotm(quat_GT_l(1)).')*(atna_GT_l(1,:,i)).';
        kf.x(16+(3*i-2):16+i*3) = atna_GT_l(1,:,i).';
    end
end

% kf.x(7:10) = compact(quaternion([cos(pi/6),sin(pi/6),0,0])).';
% kf.x(1:3) = 0.1*[0,sin(pi/3),cos(pi/3)].';
kf.P = diag(zeros(length(kf.x)-1,1));
% kf.P(1:3,1:3) =eye(3) ;

quat_KF=zeros(N,4);
pos=zeros(N,3);
vel=zeros(N,3);
atna = zeros(N,3,num_atna);
err_p=zeros(N,3); err_v=zeros(N,3); err_q=zeros(N,3);
err_ab=zeros(N,3); err_wb=zeros(N,3);
err_atna = zeros(N,3,num_atna);
z_gps=zeros(N,3,num_atna);
z_gps(1,:,1) = atna_GT_l(1,:,1);
z_gps(1,:,2) = atna_GT_l(1,:,2);
% vel_GPS = zeros(N,3,num_atna);
R_gps = zeros(3,3,num_atna);
R_gps(:,:,1) = eye(3)*power(0.5,2); % error in gps position
% R_gps(:,:,2) = eye(3)*power(0.5,2);
R_gps(:,:,2) = eye(3)*power(0.1/180*pi,2); % error in gps attitude 0.1 degrees
% R_gps(:,:,3) = eye(3)*power(0.1/180*pi,2);
bias_a = zeros(N,3); bias_w = zeros(N,3);

for i=1:N-1
    quat_KF(i,:) = kf.q;
    pos(i,:) = kf.p;
    vel(i,:) = kf.v;
    err_p(i,:) = sqrt(diag(kf.P(1:3,1:3)));
    err_v(i,:) = sqrt(diag(kf.P(4:6,4:6)));
    err_q(i,:) = sqrt(diag(kf.P(7:9,7:9))); 
    err_ab(i,:) = sqrt(diag(kf.P(10:12,10:12)));
    err_wb(i,:) = sqrt(diag(kf.P(13:15,13:15)));
    bias_a(i,:) = kf.ab;
    bias_w(i,:) = kf.wb;
    for k=1:num_atna
    atna(i,:,k) = kf.atna(:,k).';
    err_atna(i,:,k) = sqrt(diag(kf.P(15+(3*k-2):15+k*3,15+(3*k-2):15+k*3)));
    end
    z = [gyroReading(i,:),accReading(i,:)].';
%     z = [angVelBody(i,:),accBody(i,:)].';
    kf.propagate(z,dt);
    % gps signal is fused with predicted position and attitude
    if rem(i, IMU2GPS_fratio) ==0
        for k=1:num_atna
            z_gps(i+1,1:3,k) = atna_GT_l(i+1,:,k) + transpose(sqrt(R_gps(1:3,1:3,k))*randn(3,1));
            kf.update(z_gps(i+1,:,k), R_gps(:,:,k),"GPS",k);
%             kf.update(atna_GT_l(i+1,:,k), R_gps(:,:,k),"GPS",k);
        end
%         z_gps(i+1,4:7) = compact(quatmultiply(quat_GT_l(i+1,:),...
%             exp(quaternion([0,randn(1,3)/2*sqrt(R_gps(4:6,4:6))]))));
    end
end
disp("done KF "+ j)
z_gps(i+1,:,1) = atna_GT_l(i+1,:,1)+ transpose(sqrt(R_gps(1:3,1:3,1))*randn(3,1));
% non_zero_rows = ~all(z_gps==0,2);
% z_gps = z_gps(non_zero_rows,:);
% t_gps = t(non_zero_rows);
quat_KF(end,:) = kf.q.';
quat_KF = quaternion(quat_KF);
pos(i+1,:) = kf.p;
vel(i+1,:) = kf.v;
err_v(i+1,:) = sqrt(diag(kf.P(4:6,4:6)));
err_p(i+1,:) = sqrt(diag(kf.P(1:3,1:3)));
err_q(i+1,:) = sqrt(diag(kf.P(7:9,7:9)));
err_eul=quat2eul([ones(N,1),err_q/2])/pi*180;
err_atna(i+1,:,1) = sqrt(diag(kf.P(16:18,16:18)));
bias_a(i+1,:) = kf.ab;
bias_w(i+1,:) = kf.wb;
for k=1:num_atna
    atna(i+1,:,k) = kf.atna(:,k).';
end

% std_p(j,:) = std(pos - pos_GT_l);
% mean_err_p(j,:) = mean(err_p);
% std_v(j,:) = std(vel-vel_GT_l);
% mean_err_v(j,:) = mean(err_v);
% eul_KF = quat2eul(quatmultiply(quatconj(quat_KF),quat_GT_l))/pi*180;
% std_q(j,:) = std(eul_KF);
% mean_err_q(j,:) = mean(quat2eul([ones(N,1),err_q/2])/pi*180);

end
% calculate velocity of GPS measurements
% vel_GPS(:,1) = gradient(z_gps(:,1),t);
% vel_GPS(:,2) = gradient(z_gps(:,2),t);
% vel_GPS(:,3) = gradient(z_gps(:,3),t);

%% Allan Dev
data = accReading; 
tau = unique(round(logspace(0, log10(length(data)/fs/2), 1000))); % Vector of tau values where Allan variance is evaluated
avar = allanvar(data, tau, fs);
loglog(tau, sqrt(avar));
xlabel('Averaging Time \tau (s)');
ylabel('Allan Deviation (units/\surdHz)');
title('Allan Deviation of Accelerometer');
grid on;

%%
index=3;
eul_GT=quat2eul(quat_GT_l);
plot(t,[atna(:,index,2)-atna_GT_l(:,index,2),eul_GT(:,index)])
% plot(t,[vecnorm(atna(:,:,2),2,2)-3])
% plot(t,[eul_GT])
% xlim([1,10])
% std(atna(:,:,2))
%%
std(z_gps(:,:,2)-atna_GT_l(:,:,2))
% plot(t,vecnorm(atna(:,:,1)-pos,2,2))
%%
err_atna(100:150,:,2)
%%
plot(t,[quat2eul([ones(N,1),err_q/2])/pi*180])
grid on;
legend(["heading"; "pitch"; "roll"]);
title('Error in attitude')
%%
plot(t, [accReading(:,3)-g,err_ab(:,3)])

%% error in attitude
% eul_KF = quat2eul(quat_KF)/pi*180;
eul_KF = quat2eul(quatmultiply(quatconj(quat_KF),quat_GT_l))/pi*180;
std(eul_KF)
mean(err_eul)
% quat2eul([ones(N,1),err_q/2])/pi*180
%%
std(pos-pos_GT_l)
std(atna(:,1:3,1)-atna_GT_l(:,1:3,1))
% mean(err_p)
%%
std(atna(:,:,2)-atna_GT_l(:,:,2))
%%
atna(9990:10000,:,1)
atna_GT_l(9990:10000,:,1)
%%
std_p
mean_err_p
%%
rot = quat2rotm(quat_KF);
atna_local = zeros(N,3);
atna_local(1,:) = kf.atna_init_vec.';
for i=2:N
    atna_local(i,:) = rot(:,:,i).' * (atna(i,:,1)-pos(i,:)).';
end
%%
plot(t,atna_local)
%%
plot(t,err_atna)
% xlim([0,10])
%%
plot(t,z_gps-atna_GT_l)
std(z_gps-atna_GT_l)
%%
mean(err_atna)
%% 3d PLOT
f=figure(3);
f.Position=[600 300 1600 900];

% plot3(pos_GT(:,1),pos_GT(:,2),pos_GT(:,3),'LineWidth',2);

plot3(pos(:,1),pos(:,2),pos(:,3),'.','LineWidth',2);
% hold on;
% plot3(atna_GT(:,1,1),atna_GT(:,2,1),atna_GT(:,3,1),'.','LineWidth',2);
% hold off;
grid on;
xlabel('X Position',"FontSize",30);
ylabel('Y Position',"FontSize",30);
zlabel('Height',"FontSize",30);
ax = gca;
ax.FontSize = 20;
%% Plot
index=2;
f=figure(2);

f.Position=[600 300 1200 900];
errbar=[];
errbar_legend=[];
% data = [pos_raw, pos, pos_GT_l] ;
% legends = ["Raw","EKF","Ground Truth"] ;

% data = [pos, pos_GT_l] ;
% errbar = [pos_GT_l+err_p,pos_GT_l-err_p];
% errbar_legend=sprintf("Mean Error: %0.3f",mean(err_p(:,index)));
% legends = ["EKF","Ground Truth",errbar_legend] ;
% YLabel = ["x-position","y-position (m)","z-position (m)"];

% Position Error plot
% data = err_p;
% legends = ["X","Y","Z"] ;
% YLabel = ["x-position difference","y-position difference","z-position difference"];
% data = [pos - pos_GT_l];
% errbar = [err_p, -err_p];
% errbar_legend=sprintf("Mean Error: %0.3f",mean(err_p(:,index)));
% legends = [errbar_legend,"EKF"];

% data = [vel, vel_GT ];
% legends = ["EKF","GT"];

% YLabel = ["x-velocity (m/s)","y-velocity (m/s)","z-velocity (m/s)"];
% 
% data = [vel-vel_GT];
% legends = ["EKF"] ;

% quat2eul should convert to eul with order [heading pitch roll]

% Attitude plot
% eul_GT_l = quat2eul(quat_GT_l)/pi*180;
% eul_KF = quat2eul(quat_KF)/pi*180;
% errbar = [err_eul, -err_eul];
% errbar_legend=sprintf("Mean Error: %0.3f",mean(err_eul(:,index)));
% legends = [errbar_legend,"EKF","GT"];
% data = [eul_KF,eul_GT_l];
% YLabel = ["heading","pitch","roll"] ;

% data = err_eul;
% legends = ["heading","pitch","roll"] ;
% YLabel = ["Error in Attitude","Error in Attitude","Error in Attitude"];


eul_KF = quat2eul(quatmultiply(quatconj(quat_KF),quat_GT_l))/pi*180;
data = [eul_KF];
errbar = [err_eul, -err_eul];
errbar_legend=sprintf("Mean Error: %0.3f",mean(err_eul(:,index)));
legends = [errbar_legend,"EKF"];
YLabel = ["heading difference","pitch difference","roll difference"] ;

%  with raw
% eul_KF = quat2eul(quatmultiply(quatconj(quat_KF),quat_GT_l))/pi*180;
% eul_raw = quat2eul(quatmultiply(quatconj(quat_raw),quat_GT_l))/pi*180;
% data = [eul_raw eul_KF];
% legends = ["raw","EKF"];

% bias plots
% data = [accReading-accBody, bias_a];
% % data = accReading - bias_a ;
% errbar = [bias_a+err_ab,bias_a-err_ab];
% errbar_legend=sprintf("Mean Error: %0.3f",mean(err_ab(:,index)));
% legends = ["Acceleration Reading","Bias Estimate",errbar_legend];
% YLabel = ["x-acceleration (m/s^2)","y-acceleration (m/s^2)","z-acceleration (m/s^2)"];

% data = [gyroReading-angVelBody, bias_w];
% errbar = [bias_w+err_wb,bias_w-err_wb];
% errbar_legend=sprintf("Mean Error: %0.3f",mean(err_wb(:,index)));
% legends = ["Gyroscope Reading","Bias Estimate",errbar_legend];
% YLabel = ["x-rotation (rad/s)","y-rotation (rad/s)","z-rotation (rad/s)"];

Plot(t, data, errbar, index, legends)
saveas(gcf, 'plot_filename.png'); % Save as PNG
xlabel('t (sec)',"FontSize",30)
ylabel(YLabel(index),"FontSize",30)
legend("FontSize",25)
ax = gca;
ax.FontSize = 20;
grid on;
% saveas(gcf, 'KF_Sim_1_Antenna/06_gyro_bias_y.png.png'); 
% xlim([3000,3100])
% ylim([-5,5])
%% Different frequency plots, one frequency
index=1;
f=figure(2);
f.Position=[600 300 1600 900];
% data = [z_gps(:,1:3), pos, pos_GT] ;
% legends = ["GPS","KF","Ground Truth"] ;
plot(t_gps, z_gps(:,index)-pos_GT(non_zero_rows,index),'DisplayName',"GPS");
hold on;
plot(t, pos(:,index)-pos_GT(:,index),'DisplayName',"EKF");
% plot(t, pos_GT(:,index))
hold off;
YLabel = ["x-position","y-position (m)","z-position (m)"];
xlabel('t (sec)',"FontSize",30)
ylabel(YLabel(index),"FontSize",30)
legend("FontSize",25)
ax = gca;
ax.FontSize = 20;
grid on;
xlim([0,100])
%% Different frequency plots, multiple frequencies
index=2;
f=figure(2);
f.Position=[600 300 1600 900];
Fs = (1:loop_num)*10;
plot(Fs, std_q(:,index),'o-','DisplayName',"Actual Error");
hold on;
plot(Fs, mean_err_q(:,index),'o-','DisplayName',"Estimated Error");
% plot(t, pos_GT(:,index))
hold off;
% YLabel = ["x-position error","y-position error (m)","z-position error(m)"];
YLabel = ["heading error","pitch error (deg)","roll error(m)"];
% YLabel = ["x-velocity error","y-velocity error (m)","z-velocity error(m)"];
xlabel('IMU frequency (Hz)',"FontSize",30);
ylabel(YLabel(index),"FontSize",30);
legend("FontSize",25);
ax = gca;
ax.FontSize = 20;
grid on;
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

%%
% fill([t, fliplr(t)], [upper(:,index).', fliplr(lower(:,index).')], 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
%% funct get local g
function g_L = GetLocalg(quat_GT_L2G, g_G)
g_G = quaternion([0,g_G.']);
quat_GT_G2L = quatconj(quat_GT_L2G);
% q g q*
g_L = quatmultiply(quatmultiply(quat_GT_G2L, g_G),quat_GT_L2G); 
g_L = compact(g_L);
g_L = g_L(:,2:4);
end
%% funct set IMU true signal
function [accBody, angVelBody] = SetTrueIMUSignal(t, g)
N = length(t);
accBody=zeros(N,3);
angVelBody=zeros(N,3);

% angVelBody(:,1) = 1;
% disp("SetTrueIMUSignal: spin around x")

% slow motion
% % accBody(:,1) = 0.1*sin(2*pi/6000*t);
% % accBody(:,2) = 0.1*cos(2*pi/6000*t);
% % accBody(:,3) = 0.05*sin(2*pi/1000*t)+0.05*sin(2*pi/200*t);

angVelBody(:,3) = 0.2*sin(2*pi/100*t)+0.4*sin(2*pi/5000*t); % T=115.2s/5184s
angVelBody(:,2) = 0.001*sin(2*pi/1200*t);
angVelBody(:,1) = 0.001*sin(2*pi/1200*t);
% disp("SetTrueIMUSignal: slow motion");

% fast motion, IMU sampling frequency needs to be big enough to get right
% estimation
% accBody(:,1) = 10*sin(2*pi/60*t);
% accBody(:,2) = 10*cos(2*pi/60*t);
% accBody(:,3) = 5*sin(2*pi/10*t)+5*sin(2*pi/2*t);
% 
% angVelBody(:,3) = 20*sin(2*pi/1*t)+20*sin(2*pi/50*t);
% angVelBody(:,2) = 0.1*sin(2*pi/12*t);
% angVelBody(:,1) = 0.1*sin(2*pi/12*t);
% disp("SetTrueIMUSignal: fast motion");

% pdm = Pendulum();
% pdm.l = 0.1 ; pdm.m = 1.0 ; pdm.w = 0;
% pdm.theta = pi/3 ; pdm.g = g;
% dt = t(2) - t(1);
% for i=1:N
%     accBody(i,2) = pdm.ax ;
%     accBody(i,3) = pdm.ay ;
% %     angVelBody(i,1) = pdm.w ;
%     pdm.propagate(dt);
% end
% disp("SetTrueIMUSignal: pendulum motion")

% accBody(:,1) = 10;
% angVelBody(:,1) = 0.1;
% angVelBody(:,2) = 0.1;
% angVelBody(:,3) = 0.1;
end
%% funct plot
function Plot(t, data, errbar, index, legends)
[~,col_num] = size(data);
data_num = round(col_num/3);
if ~isempty(errbar)
    upper = errbar(:,1:3); lower = errbar(:,4:6);
    h=fill([t, fliplr(t)], [upper(:,index).', fliplr(lower(:,index).')], 'r', ...
        'FaceAlpha', 0.3,'EdgeColor','none');
%     set(h, 'HandleVisibility', 'off');
end
hold on;
j = index;
for i=1:data_num
    plot(t, data(:,j))
    j = j + 3;
end
% if ~isempty(errbar)
%     upper = errbar(:,1:3); lower = errbar(:,4:6);
%     h=fill([t, fliplr(t)], [upper(:,index).', fliplr(lower(:,index).')], 'r', ...
%         'FaceAlpha', 0.3,'EdgeColor','none');
% %     set(h, 'HandleVisibility', 'off');
% end
hold off;
legend(legends);
end
%%
% function R_gps=z_atna_cov(quat,atna,pos_err, angle_err)
% % input: current attitude; initial atna vector in local frame.
% % uncertainty in main antenna position
% % uncertainty in pointing angle in radians
% num_atna = size(atna,2);
% l = atna(:,2)-atna(:,1);
% l = quat2rotm(quat)*l;
% [phi,el,r]=cart2sph(l(1),l(2),l(3));
% theta = pi/2 - el;
% R_gps = eye(num_atna*3);
% R_gps(1:3,1:3) = R_gps(1:3,1:3)*pos_err^2;
% e11=1+cos(2*(theta+phi)); e12=sin(2*(theta+phi)); e13=sin(phi)-sin(2*theta+phi);
% e21=sin(2*(theta+phi)); e22=1-cos(2*(theta+phi)); e23=-cos(phi)+cos(2*(theta+phi));
% e31=sin(phi)-sin(2*(theta+phi)); e32=-cos(phi)+cos(2*(theta+phi)); e33=2*sin(theta)^2;
% cov=[e11,e12,e13;e21,e22,e23;e31,e32,e33];
% % R_gps(1:3,4:6) = R_gps(1:3,1:3);
% % R_gps(4:6,1:3) = R_gps(1:3,4:6).';
% % R_gps(4:6,4:6) = R_gps(1:3,1:3) + norm(l)^2 * angle_err^2/2 * cov;
% R_gps(4:6,4:6) = R_gps(1:3,1:3);
% end
%% funct quaternion interpolation
function qi = Slerp(t1, q, t2)
% frac = zeros(length(t2));
q = compact(q);
qi = zeros(length(t2),4);
for i = 1:length(t2)
    % Find the closest indices in t1 that are less than or equal to and greater than t2(i)
    idx_below = find(t1 <= t2(i), 1, 'last');
    idx_above = find(t1 >= t2(i), 1, 'first');

    % Check for boundary conditions
    if isempty(idx_below)  % t2(i) is before the first t1
        disp "outside of bounds" ;  % Default to 0 if out of bounds
        break;
    elseif isempty(idx_above)  % t2(i) is after the last t1
        disp "outside of bounds";  % Default to 1 if out of bounds
        break;
    elseif idx_below == idx_above  % t2(i) exactly matches a t1
        qi(i,:) = q(idx_below,:);  % Exact match (or could set to 1 depending on context)
    else
        % Linear fraction calculation
        t1_below = t1(idx_below);
        t1_above = t1(idx_above);
        q_below = q(idx_below,:);
        q_above = q(idx_above,:);
        frac = (t2(i) - t1_below) / (t1_above - t1_below);
        qi(i,:) = quatinterp(q_below, q_above, frac);
    end
end
qi = quaternion(qi);
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
%%