global t;global ti;global ts;global num_of_interval;
folder_name='Sep13/';
abx_file='abx_two.txt'; apx_file='apx_18.txt';
abx_file=strcat(folder_name,abx_file);
apx_file=strcat(folder_name,apx_file);
num_of_interval=8;
ti=zeros(num_of_interval*2,1);
ti(1)=191703;ti(2)=191806;
ti(3)=192042;ti(4)=192404;
ti(5)=193550;ti(6)=193648;
ti(7)=192934;ti(8)=193254;
ti(9)=195216;ti(10)=195322;
ti(11)=195607;ti(12)=195928;
ti(13)=200258;ti(14)=200353;
ti(15)=200557;ti(16)=200921;

ts=zeros(2,1); % static time interval
ts(1)=203044+2;ts(2)=204300-2;
% ts(3)=203044+2;ts(4)=204300-2;
% ts(5)=201226+2;ts(6)=202818-2;
[t,abx_hpr,apx_hpr]=Read_in(abx_file,apx_file);
[ti,ts]=Set_time_interval();
backup=[t,abx_hpr,apx_hpr];
Change_t_to_datetime();
Change_t_to_seconds();
% apx_hpr=Align_apx_with_abx(abx_hpr,apx_hpr,1); % last input put in the start index of ts you wish to use for alignment
d_hpr=Find_difference_between_abx_and_apx(abx_hpr,apx_hpr);
%% Plot heading pitch roll
Title=["heading","pitch","roll"];
figure(1);
k=1;
for i=1:3
    subplot_tight(6,1,k,0.05);
    k=k+1;
    plot(t,abx_hpr(:,i));
    hold on ;
    plot(t,apx_hpr(:,i),'--');
    hold off;
    title(Title(i));
    ylabel("degree");
    xlabel('time');
%     xlim([t(min(ti)),t(max(ti))]);
    xlim([t(1),t(max(ti))]);
%     xlim([t(ts(5)),t(ts(4))]);
%     Set_ylim(i,[240,260],[-2,2],[-1,1]);
    for j=1:length(ti) % set xline and its color
        if rem(j,2)==1
            xline(t(ti(j)),'g');
        else
            xline(t(ti(j)),'m');
        end
    end
    legend('abx','apx','start','stop');
    subplot_tight(6,1,k,0.05);
    k=k+1;
    plot(t,d_hpr(:,i),'.','MarkerSize',3);
    xlabel('time');
%     xlim([t(min(ti)),t(max(ti))]);
    xlim([t(1),t(max(ti))]);
%     xlim([t(ts(5)),t(ts(4))]);
    Set_ylim(i,[-0.5,0.5],[-0,1],[-3,-2]);
    for j=1:length(ti)
        if rem(j,2)==1
            xline(t(ti(j)),'g');
        else
            xline(t(ti(j)),'m');
        end
    end
    legend('difference','start','stop');
    temp =Title(i)+" defference";
    title(temp);
    ylabel("degree");
end
%% Plot histogram
Histogram_of_d_hpr_by_interval(d_hpr);
%% Make a table of mean and std
Table_of_mean_and_std(d_hpr);
%% match hpr with histogram
Match_hpr_with_their_histogram(abx_hpr,apx_hpr,d_hpr);
%% Generate random normal distribution
Generate_rand_normal_distribution(d_hpr);
%% Reset
[t,abx_hpr,apx_hpr]=Reset(backup);
apx_hpr=Align_apx_with_abx(abx_hpr,apx_hpr,1);
d_hpr=Find_difference_between_abx_and_apx(abx_hpr,apx_hpr);
Change_t_to_datetime();
% Change_t_to_seconds();
%% Write table element to a file
row_number=4;
np_array=Write_table_element_of_mean_and_std(d_hpr,row_number);
%% Print time stamp
format long g;
disp([t(ts)]);
%% Read Accelerometer
Accelerometer = readmatrix("August05/accel1.log");
A=Accelerometer(:,2); %reads which accelerometer
at=Accelerometer(:,1);
ax=Accelerometer(:,3);
ay=Accelerometer(:,4);
az=Accelerometer(:,5);
j=1;
k=1;
at0=[];at1=[];
ax0=[];ax1=[];
ay0=[];ay1=[];
az0=[];az1=[];
for i=1:length(A)
    if A(i)==0
        at0(j) = at(i);
        ax0(j) = ax(i);
        ay0(j) = ay(i);
        az0(j) = az(i);
        j=j+1;
    else
        at1(k) = at(i);
        ax1(k) = ax(i);
        ay1(k) = ay(i);
        az1(k) = az(i);
        k=k+1;
    end
end
% mean(ax1)^2 + mean(ay1)^2 + mean(az1)^2
% mean(ax0)^2 + mean(ay0)^2 + mean(az0)^2
%% plot accelerometer
figure(7);
subplot(3,1,1);
plot(at0,ax0);
subplot(3,1,2);
plot(at0,ay0);
subplot(3,1,3);
plot(at0,az0)
%% 
% subplot(3,1,1);
% plot(t(ti(1):ti(2)),abx_hpr(ti(1):ti(2),2))
% subplot(3,1,2);
% a=fft(abx_hpr(ti(1):ti(2),2));
% plot(t(ti(1):ti(2)),a,'.');

Fs=length(t(min(find(t>4500)):max(find(t<5000))))/500;
figure(6);
highpass(abx_hpr(min(find(t>4500)):max(find(t<5000)),3),0.5,Fs);
figure(7);
highpass(apx_hpr(min(find(t>4500)):max(find(t<5000)),3),0.5,Fs);

%%
timin=min(find(t>4300));
timax=max(find(t<5300));
% timin=ti(3);
% timax=ti(4);
fs=(timax-timin)/(t(timax)-t(timin));
range1=d_hpr(timin:timax,3);
% range1=abx_hpr(timin+1:timax,3);
% range2=apx_hpr(timin+1:timax,3);
figure(9);
y_filt1=Highpass_filter(range1,0.1,fs);
% figure(10)
% y_filt2=Highpass_filter(range2,0.1,fs);

% figure(12);
% subplot(2,1,1);
% histfit(range1);
% pd = fitdist(range1,'Normal');
% Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
% legend('',Legend);
% % xlim([0.26,0.38]);
% subplot(2,1,2);
% histfit(abs(y_filt1));
% pd = fitdist(abs(y_filt1),'Normal');
% Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
% legend('',Legend);
% xlim([0.26,0.38]);
% figure(13);
% 
% subplot(2,1,1);
% histfit(range2);
% pd = fitdist(range2,'Normal');
% Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
% legend('',Legend);
% xlim([-0.28,-0.18]);
% subplot(2,1,2);
% histfit(abs(y_filt2));
% pd = fitdist(abs(y_filt2),'Normal');
% Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
% legend('',Legend);
% xlim([0.18,0.28]);

%% funct Highpass filter
function [y_filt_shift]=Highpass_filter(y_ori,fpass,fs)
n=0:length(y_ori)-1;
N=length(n);
x=n/fs;
time_domain=zeros(N/2,1);
y=zeros(N/2,1);
j=1 ;
for i=1:length(x)-1
    if rem(i,2)==1
        time_domain(j)=x(i);
        y(j)=y_ori(i);
        j=j+1;
    end
end
frequency_domain=0:fs/N:(fs-fs/N)/2;
y_fft=fft(y_ori);
y_fft=y_fft(1:end/2);
y_fft_filt=y_fft;
for i=1:max(find(frequency_domain<fpass))
    y_fft_filt(i)=y_fft_filt(i)/100;
end
y_filt=ifft(y_fft_filt);
y_filt_shift=y_filt+mean(y)-mean(y_filt);
% Together
subplot(2,1,1);
plot(time_domain,y);
hold on;
if(mean(y)<0)
    plot(time_domain,-abs(y_filt_shift))
else
    plot(time_domain,abs(y_filt_shift));
hold off;
end
legend('original','filtered');

% xlim([0,50]);
subplot(2,1,2);
plot(frequency_domain,log10(abs(y_fft)));
hold on;
plot(frequency_domain,log10(abs(y_fft_filt)));
hold off;
xlim([min(frequency_domain),max(frequency_domain)]);
legend('original','filtered');
% Separate plots
% figure(10);
% subplot(2,1,1);
% plot(time_domain,y);
% % xlim([0,10]);
% subplot(2,1,2);
% plot(frequency_domain,log10(abs(y_fft)));
% xlim([min(frequency_domain),max(frequency_domain)]);
% 
% figure(11);
% subplot(2,1,1);
% plot(time_domain,abs(y_filt_shift));
% % xlim([0,10]);
% subplot(2,1,2);
% plot(frequency_domain,log10(abs(y_fft_filt)));
% xlim([min(frequency_domain),max(frequency_domain)]);
end
%% funct Generate Normal random distribution
function hist_hpr=Generate_rand_normal_distribution(d_hpr)
global ti;
global num_of_interval;
for j=1:1
    figure(j+3);
    for i=1:num_of_interval
        subplot(num_of_interval/2,2,i);
        if rem(i,2)==1
            Title='fast';
        else
            Title='slow';
        end
        a=d_hpr(ti(2*i-1):ti(2*i),j);
        d_range=normrnd(mean(a),std(a),length(a),1);
        histfit(d_range);
        pd = fitdist(d_range,'Normal');
        Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
        legend('',Legend);
        if j==1
            xlim([-0.15,0.2]);
            ylim([0,250]);
        elseif j==2
            xlim([-0.15,0.25]);
            ylim([0,200]);
        else
            xlim([-0.3,0.2]);
            ylim([0,200]);
        end
        title(Title);
        if i==1 | i==3
            ylabel('Adaptive')
        end
        if i==5
            ylabel('Quasi-static');
        end
        if i==7
            ylabel('Walking');
        end
    end
end
end


%% funct match with histogram
function []=Match_hpr_with_their_histogram(abx_hpr,apx_hpr,d_hpr)
global num_of_interval;
global t;
global ti;
Label=["Heading";"Pitch";"Roll"];
for i=1:num_of_interval
    figure(i);
    for j=1:3
        subplot(3,2,2*j-1);
        if rem(i,2)==1
            Title='fast';
        else
            Title='slow';
        end
        d_range=d_hpr(ti(2*i-1):ti(2*i),j);
        histfit(d_range);
        pd = fitdist(d_range,'Normal');
        Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
        legend('',Legend);
        ylabel(Label(j));
        if j==1
            title(Title);
        end
        subplot(3,2,2*j);
        plot(t(ti(2*i-1):ti(2*i)),d_hpr(ti(2*i-1):ti(2*i),j),'.','MarkerSize',j);
    end

end
end
%%
function np_array=Write_table_element_of_mean_and_std(d_hpr,row_number)
global num_of_interval;
global ti;
for j=1:3
    np_array=zeros(row_number,6);
    Average=zeros(num_of_interval,1);
    Average_Error=zeros(num_of_interval,1);
    Sigma=zeros(num_of_interval,1);
    for i=1:num_of_interval
        Average(i)=mean(d_hpr(ti(2*i-1):ti(2*i),j));
        Sigma(i) = std(d_hpr(ti(2*i-1):ti(2*i),j)) ;
        Average_Error(i)=Sigma(i)/sqrt(length(d_hpr(ti(2*i-1):ti(2*i),j)));
    end
    l=1;
    for i=1:row_number
        for k=1:2
            np_array(i,3*k-2)=Average(l);
            np_array(i,3*k-1)=Average_Error(l);
            np_array(i,3*k)=Sigma(l);
            l=l+1;
        end
    end
fileID=["Sep13/table_h.txt","Sep13/table_p.txt","Sep13/table_r.txt"];
file1=fopen(fileID(j),'w');
fprintf(file1,'%.15g,%.15g,%.15g,%.15g,%.15g,%.15g\r\n',np_array.');
fclose(file1);
end

end
%% funct table of mean and std
function tab=Table_of_mean_and_std(d_hpr)
global num_of_interval;
global ti;
FilterSetting=["Adaptive";"Adaptive";"Quasi-static";"Walking"];
for j=1:3
    Average=zeros(num_of_interval,1);
    Average_Error=zeros(num_of_interval,1);
    Sigma=zeros(num_of_interval,1);
    for i=1:num_of_interval/2
        Average(i)=mean(d_hpr(ti(4*i-3):ti(4*i-2),j));
        Sigma(i)=std(d_hpr(ti(4*i-3):ti(4*i-2),j));
        Average_Error(i)=Sigma(i)/length(d_hpr(ti(4*i-3):ti(4*i-2),j));
    end
    for i=num_of_interval/2+1:num_of_interval
        Average(i)=mean(d_hpr(ti(4*i-17):ti(4*i-16),j));
        Sigma(i)=std(d_hpr(ti(4*i-17):ti(4*i-16),j));
        Average_Error(i)=Sigma(i)/length(d_hpr(ti(4*i-17):ti(4*i-16),j));
    end
    table(FilterSetting,Average(1:num_of_interval/2),Average_Error(1:num_of_interval/2),Sigma(1:num_of_interval/2),Average(5:8),Average_Error(5:8),Sigma(5:8))
end
end
%% funct Plot histogram of d_hpr
function hist_hpr=Histogram_of_d_hpr_by_interval(d_hpr)
global ti;
global num_of_interval;
for j=1:3
    figure(j+3);
    for i=1:num_of_interval
        subplot(num_of_interval/2,2,i);
        if rem(i,2)==1
            Title='fast';
        else
            Title='slow';
        end
        d_range=d_hpr(ti(2*i-1):ti(2*i),j);
        histfit(d_range);
        pd = fitdist(d_range,'Normal');
        Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
        legend('',Legend);
        if j==1
            xlim([-0.15,0.2]);
            ylim([0,150]);
        elseif j==2
            xlim([0.2,0.8]);
            ylim([0,150]);
        else
            xlim([-3,-2.4]);
            ylim([0,150]);
        end
        title(Title);
        if i==1 | i==3
            ylabel('Adaptive')
        end
        if i==5
            ylabel('Automobile');
        end
        if i==7
            ylabel('Walking');
        end
    end
end
end
%% funct Reset t,abx_hpr,apx_hpr to intial values
function [t,abx_hpr,apx_hpr]=Reset(backup)
t=backup(:,1);
abx_hpr=backup(:,2:4);
apx_hpr=backup(:,5:7);
end
%% function Set xlim
function []=Set_ylim(i,lim_1,lim_2,lim_3)
if i==1
        ylim(lim_1);
elseif i==2
        ylim(lim_2);
else
        ylim(lim_3);
end
end
%% funct Plot heading pitch roll
function plt_hpr = Plot_heading_pitch_roll(abx_hpr,apx_hpr,d_hpr)
global t;global ti;global ts;
Title=["heading","pitch","roll"];
for i=1:3
    figure(i);
    subplot(2,1,1);
    plot(t,abx_hpr(:,i));
    hold on ;
    plot(t,apx_hpr(:,i),'--');
    hold off;
    ylabel('degree');
    xlabel('time');
%     xlim([t(min(ti)),t(max(ti))]);
    xlim([t(1),t(max(ti))]);
%     xlim([t(ts(5)),t(ts(4))]);
    title(Title(i));
    for j=1:length(ti) % set xline and its color
        if rem(j,2)==1
            xline(t(ti(j)),'g');
        else
            xline(t(ti(j)),'m');
        end
    end
    legend('abx','apx','start','stop');
    subplot(2,1,2);
    plot(t,d_hpr(:,i),'.','MarkerSize',3);
    ylabel('dR_{bp}');
    xlabel('time');
%     xlim([t(min(ti)),t(max(ti))]);
    xlim([t(1),t(max(ti))]);
%     xlim([t(ts(5)),t(ts(4))]);
    if i==1
        ylim([-0.3,0.3]);
    elseif i==2
        ylim([0,1]);
    else
        ylim([-3,3]);
    end
    for j=1:length(ti)
        if rem(j,2)==1
            xline(t(ti(j)),'g');
        else
            xline(t(ti(j)),'m');
        end
    end
    legend('difference','start','stop');
end
end
%% funct Align apx with abx
function apx_hpr=Align_apx_with_abx(abx_hpr,apx_hpr,ts_start)
global t;global ti;global ts;
abx_h=abx_hpr(:,1);abx_p=abx_hpr(:,2);abx_r=abx_hpr(:,3);
apx_h=apx_hpr(:,1);apx_p=apx_hpr(:,2);apx_r=apx_hpr(:,3);
for i=ts_start:ts_start
    m_abx_h=mean(abx_h(ts(i):ts(i+1))); % average heading during the static time
    m_abx_p=mean(abx_p(ts(i):ts(i+1)));
    m_abx_r=mean(abx_r(ts(i):ts(i+1)));
    m_apx_h=mean(apx_h(ts(i):ts(i+1)));
    m_apx_p=mean(apx_p(ts(i):ts(i+1)));
    m_apx_r=mean(apx_r(ts(i):ts(i+1)));
    m_Rb = R(m_abx_h,m_abx_p,m_abx_r);
    m_Rp = R(m_apx_h,m_apx_p,m_apx_r);
    m_dR = m_Rb*m_Rp.';
end
for i=1:length(t)
    Rp=R(apx_h(i),apx_p(i),apx_r(i));
    Rp=m_dR*Rp;
    temp=get_angle(Rp);
    if temp(1)<=0
        temp(1)=temp(1)+360;
    end
    apx_hpr(i,1)=temp(1);
    apx_hpr(i,2)=temp(2);
    apx_hpr(i,3)=temp(3);
end
end
%% funct Change t to seconds; input t has to be datetime
function t=Change_t_to_seconds();
global t; global ti;
temp=zeros(length(t),1);
for i=1:length(t)
    a=diff([t(ti(1)),t(i)]);
    temp(i)=seconds(a);
end
t=temp;
end
%% funct Change t to datetime; input t has to be double
function t=Change_t_to_datetime()
global t;
t=num2str(t,'%.3f');
t=datetime(t,"Format",'HHmmss.SSS');
end
%% funct Set time interval
function [ti,ts]=Set_time_interval()
global t;global ti;global ts;global num_of_interval;

% add 2 sec to start time and minus 2sec to the end time
% make ti to be the index of the time interval.
for i=1:num_of_interval*2
    if rem(i,2)==1
        ti(i)=ti(i)+2;
    else
        ti(i)=ti(i)-2;
    end
    ti(i)=min(find(t>=ti(i)));
    if i<=length(ts)
        ts(i)=min(find(t>=ts(i)));
    end
end
end
%% funct Find difference between abx and apx
function d_hpr=Find_difference_between_abx_and_apx(abx_hpr,apx_hpr)
global t;
d_heading=ones(length(t),1);
d_pitch=ones(length(t),1);
d_roll=ones(length(t),1);
abx_h=abx_hpr(:,1);abx_p=abx_hpr(:,2);abx_r=abx_hpr(:,3);
apx_h=apx_hpr(:,1);apx_p=apx_hpr(:,2);apx_r=apx_hpr(:,3);
for i=1:length(t)
    R_b = R(abx_h(i),abx_p(i),abx_r(i));
    R_p = R(apx_h(i),apx_p(i),apx_r(i));
    dR = R_b*R_p.';
    temp=get_angle(dR);
    d_heading(i) = temp(1);
    d_pitch(i) = temp(2);
    d_roll(i) = temp(3);
end
d_hpr=[d_heading,d_pitch,d_roll];
end
%% funct Read in
function [t,abx_hpr,apx_hpr]=Read_in(abx_data,apx_data)
abx = readtable(abx_data);
apx = readtable(apx_data);
% format long g
abx_attitude_time = abx(:,3);
abx_heading = abx(:,4);
abx_pitch = abx(:,5);
abx_roll = abx(:,6);
abx_attitude_time = abx_attitude_time{:,:};
abx_heading = abx_heading{:,:};
abx_pitch = abx_pitch{:,:};
abx_roll = abx_roll{:,:};

apx_attitude_time = apx(:,2);
apx_heading = apx(:,3);
apx_pitch = apx(:,6);
apx_roll = apx(:,5);
apx_attitude_time = apx_attitude_time{:,:};
apx_heading = apx_heading{:,:};
apx_pitch = apx_pitch{:,:};
apx_roll = apx_roll{:,:};
t=[];
abx_h=[];
abx_p=[];
abx_r=[];
apx_h=[];
apx_p=[];
apx_r=[];
for i=1:length(abx_attitude_time)
    for j=1:length(apx_attitude_time)
        if abx_attitude_time(i)==apx_attitude_time(j)
            if isnan(abx_heading(i)) | isnan(apx_heading(i))
                continue
            else
                t(end+1) = abx_attitude_time(i);
                abx_h(end+1) = abx_heading(i);
                abx_p(end+1) = abx_pitch(i);
                abx_r(end+1) = abx_roll(i);

                apx_h(end+1) = apx_heading(j);
                apx_p(end+1) = apx_pitch(j);
                apx_r(end+1) = apx_roll(j);
            end
        end
    end
end

t=transpose(t);
abx_h=transpose(abx_h);
abx_p=transpose(abx_p);
abx_r=transpose(abx_r);
apx_h=transpose(apx_h);
apx_p=transpose(apx_p);
apx_r=transpose(apx_r);

% for i=1:length(t)
%     if apx_h(i)-abx_h(i)>350
%         apx_h(i)= apx_h(i)-360 ;
%     end
%     if apx_h(i)-abx_h(i)<-350
%         abx_h(i)=abx_h(i)-360 ;
%     end
% end
abx_hpr=[abx_h,abx_p,abx_r];
apx_hpr=[apx_h,apx_p,apx_r];
end