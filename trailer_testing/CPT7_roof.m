[file1]=Read_in("23Feb01/feb1_long_rooftest.txt");
file1(:,1)=file1(:,1)-file1(1,1);
t=file1(:,1);
file1_hpr=file1(:,2:4);
%%
find(file1_hpr(:,1)>360)
file1_hpr(11379,1)
%% Check if INS Status is GOOD
N=height(file);
n=0;
l=0;
for i=1:N
    if contains(string(file{i,15}),"GOOD")
        l=l+1;
    else
        n=n+1;
    end
end
l
n
%% Plot heading pitch roll
Title=["heading","pitch","roll"];
for i=1:3
    f=figure(i);
    f.Position=[500 200 1600 900];
    plot(t,file1_hpr(:,i),'--');
    ylabel(Title(i) + " (deg.)","FontSize",11);
    xlabel('time (s)',"FontSize",11);
    title(Title(i));
    ax = gca;
    ax.FontSize = 12;
    grid on;
%     if i==2
%         ylim([0.4,0.6])
%     end
%     if i==3
%         ylim([1.1,1.3])
%     end
end
%% Plot histogram with Gaussian fit
t1=min(find(t>2e4));
% t2=max(find(t<14e4));

hpr_index=1;
Title=["heading","pitch","roll"];
f=figure(4);
f.Position=[500 200 1600 900];
% Xlim=[283.1,283.35;0.45,0.6;1.16,1.24];
Xlim=[];
for i=1:3
    subplot(3,1,i);
    Plot_histogram(file1_hpr(t1:end,i),Title(i),Xlim,"degree","CPT7");
end
%%
% t1=1;
% t2=length(t);


%% plot function in time and frequency domain
% funct=2*randn(size(funct));
t1=min(find(t>10e4));
t2=max(find(t<14e4));
fs=(t2-t1)/(t(t2)-t(t1));
Title=["heading","pitch","roll"];
for i=3:3
    f=figure(i);
    f.Position=[500 200 800 450];
    funct=file1_hpr(t1:t2,i);
    N=length(funct);
    frequency_domain=0:fs/N:(fs-fs/N)/2;
    y=Fourier_transform(funct,fs);
    subplot(2,1,1);
    plot(t(t1:t2),funct)
    ylabel(Title(i) + " (deg.)","FontSize",11);
    xlabel('time (s)',"FontSize",11);
    title(Title(i));
    ax = gca;
    ax.FontSize = 12;
    grid on;
    xlim([120000,120100])
    subplot(2,1,2);
    plot(frequency_domain,y);
    % plot settings
    ylabel("dB","FontSize",11);
    xlabel('Frequency (Hz)',"FontSize",11);
    ax = gca;
    ax.FontSize = 12;
    grid on;
    ylim([-40,20])
end
%% Plot phase of the fourier tansform and correlation function

Title=["heading","pitch","roll"];
t1=min(find(t>2e4));
t2=max(find(t<6e4));
fs=(t2-t1)/(t(t2)-t(t1));
hpr_index=2;
funct=file1_hpr(t1:t2,hpr_index);
y_fft1=fft(funct);
y_fft1=y_fft1(1:end/2 + rem(length(funct),2) )/2;
hpr_index=3;
funct=file1_hpr(t1:t2,hpr_index);
y_fft2=fft(funct);
y_fft2=y_fft2(1:end/2 + rem(length(funct),2) )/2;
f=figure(4);
f.Position=[500 200 1600 900];
% plot(frequency_domain,angle(y_fft1));
% hold on;
% plot(frequency_domain,angle(y_fft2));
% hold off;
% xlim([0.5,0.502]);
% ylabel("Frequency","FontSize",11);
% xlabel('Phase',"FontSize",11);
% legend("Pitch phase","Roll phase",'FontSize',14);

ax = gca;
ax.FontSize = 12;


phase1=angle(y_fft1);
phase2=angle(y_fft2);
[c,lags]=xcorr(phase1,phase2,length(phase1)*0.05,'normalized');

plot(lags/10,c);
title("Correlation between pitch and roll phases","FontSize",14);
xlabel('Lag',"FontSize",14);
ylabel("Correlation","FontSize",14);
ylim([-0.01,0.06])
grid on;
%%

%%
temp =0:0.01:10000;
x1 = randn(size(temp));
x2 = randn(size(temp));
% x1=sin(temp);
% x2=sin(temp);
[c,lags]=xcorr(x1,x2,1000,'normalized');
% c=c/(rms(x1)*rms(x2))/length(temp);
plot(lags,c);
ylabel("Lag","FontSize",11);
xlabel('Correlation',"FontSize",11);
ax = gca;
ax.FontSize = 12;


%% plot function in time and frequency domain after filtering
[time_domain,y_filt]=Highpass_filter(funct,0.5,fs);
time_domain=time_domain+10e4;
f=figure(2);
f.Position=[500 200 1600 900];
subplot(2,1,1);
plot(time_domain,y_filt);
xlim([120059,120064]);
ylabel(Title(1) + " (deg.)","FontSize",11);
xlabel('time (s)',"FontSize",11);
title(Title(1));
ax = gca;
ax.FontSize = 12;
grid on;
subplot(2,1,2);
plot(frequency_domain,y)
ylabel("dB","FontSize",11);
xlabel('Frequency (Hz)',"FontSize",11);
ax = gca;
ax.FontSize = 12;
grid on;
%% Compare 2 fft plots at different time periods
hpr_index=3;
Title=["heading","pitch","roll"];
ti=zeros(4,1);
ti(1)=min(find(t>2e4)); ti(2)=max(find(t<6e4));
ti(3)=min(find(t>6e4)); ti(4)=max(find(t<10e4));
for i=1:4
f=figure(1);
f.Position=[500 200 1600 900];
funct=file1_hpr(ti(1):ti(2),hpr_index);
N=length(funct);
frequency_domain=0:fs/N:(fs-fs/N)/2;
y=Fourier_transform(funct,fs);
subplot(2,1,1);
plot(frequency_domain,y);
% plot settings
ylabel("dB","FontSize",11);
xlabel('Frequency (Hz)',"FontSize",11);
ax = gca;
ax.FontSize = 12;
grid on;
ylim([-40,20]);
title(Title(hpr_index));

subplot(2,1,2);
funct=file1_hpr(ti(3):ti(4),hpr_index);
y=Fourier_transform(funct,fs);
plot(frequency_domain,y);
% plot settings
ylabel("dB","FontSize",11);
xlabel('Frequency (Hz)',"FontSize",11);
ax = gca;
ax.FontSize = 12;
grid on;
ylim([-40,20]);
end

%% funct Correlation function


%% funct Highpass filter
function [time_domain,y_filt_shift]=Highpass_filter(y_ori,fpass,fs)
n=0:length(y_ori)-1;
N=length(n);
x=n/fs;

time_domain=zeros((N+rem(N,2))/2,1);
y=zeros((N+rem(N,2))/2,1);
j=1 ;
for i=1:length(x)+rem(length(x),2)
    if rem(i,2)==1
        time_domain(j)=x(i);
        y(j)=y_ori(i);
        j=j+1;
    end
end
frequency_domain=0:fs/N:(fs-fs/N)/2;
y_fft=fft(y_ori);
y_fft=y_fft(1:end/2+rem(length(y_fft),2));
y_fft_filt=y_fft;
for i=1:max(find(frequency_domain<fpass))
    y_fft_filt(i)=y_fft_filt(i)/100;
end
y_filt=ifft(y_fft_filt);
y_filt_shift=y_filt+mean(y)-mean(y_filt);



% Together
% subplot(2,1,1);
% plot(time_domain,y);
% hold on;
% if(mean(y)<0)
%     plot(time_domain,-abs(y_filt_shift))
% else
%     plot(time_domain,abs(y_filt_shift));
% hold off;
% end
% legend('original','filtered');
% 
% 
% subplot(2,1,2);
% plot(frequency_domain,log10(abs(y_fft)));
% hold on;
% plot(frequency_domain,log10(abs(y_fft_filt)));
% hold off;
% xlim([min(frequency_domain),max(frequency_domain)]);
% legend('original','filtered');

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
%% funct Lowpass filter
function [time_domain,y_filt_shift]=Lowpass_filter(y_ori,fpass,fs)
n=0:length(y_ori)-1;
N=length(n);
x=n/fs;

time_domain=zeros((N+rem(N,2))/2,1);
y=zeros((N+rem(N,2))/2,1);
j=1 ;
for i=1:length(x)+rem(length(x),2)
    if rem(i,2)==1
        time_domain(j)=x(i);
        y(j)=y_ori(i);
        j=j+1;
    end
end
frequency_domain=0:fs/N:(fs-fs/N)/2;
y_fft=fft(y_ori);
y_fft=y_fft(1:end/2+rem(length(y_fft),2));
y_fft_filt=y_fft;
for i=min(find(frequency_domain>fpass)):max(find(frequency_domain<fpass+0.1))
    y_fft_filt(i)=y_fft_filt(i)/100;
end
y_filt=ifft(y_fft_filt);
y_filt_shift=y_filt+mean(y)-mean(y_filt);

% subplot(2,1,1);
% plot(time_domain,y);
% hold on;
% if(mean(y)<0)
%     plot(time_domain,-abs(y_filt_shift))
% else
%     plot(time_domain,abs(y_filt_shift));
% hold off;
% end
% legend('original','filtered');
% 
% 
% subplot(2,1,2);
% plot(frequency_domain,log10(abs(y_fft)));
% hold on;
% plot(frequency_domain,log10(abs(y_fft_filt)));
% hold off;
% xlim([min(frequency_domain),max(frequency_domain)]);
% legend('original','filtered');
end
%% funct Fourier Transform
function [y]=Fourier_transform(funct,fs)

y_fft=fft(funct);
y_fft=y_fft(1:end/2 + rem(length(funct),2) )/2;
% plot(frequency_domain,log10(abs(y)));
y=10*log10(abs(y_fft));

end
%% funct Plot histogram
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
%%
function [file1]=Read_in(file_name)

file = readtable(file_name);
N=height(file);
file1=zeros(N,4);
n=0;
for i=1:N
    if string(file{i,1}) == '#INSATTA' & file{i,11}>=0.1 & isnan(file{i,14}) ~= 1
        n=n+1;
        file1(n,:)=[file{i,11},file{i,14},file{i,13},file{i,12}];
    end
end
file1=file1(1:n,:);
end