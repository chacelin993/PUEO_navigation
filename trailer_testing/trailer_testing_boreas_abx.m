[t,file1_hpr,file2_hpr]=Read_in("23Jan11/jan11_morning_system_state.txt","23Jan11/ABX-two.txt");
num_of_interval=6;
ti=zeros(num_of_interval*2,1);
%human
% ti(1)=210222;ti(2)=210540;
% ti(3)=210727;ti(4)=211050;
% ti(5)=211220;ti(6)=211527;
% ti(7)=211721;ti(8)=212045;
% ti(9)=212231;ti(10)=212533;
% ti(11)=212711;ti(12)=213029;

% ti(1)=201314;ti(2)=201640;

% mourning run:
ti(1)=165009;ti(2)=165327;
ti(3)=165530;ti(4)=165930;
ti(5)=170154;ti(6)=170518;
ti(7)=170724;ti(8)=171137;
ti(9)=171331;ti(10)=171643;
ti(11)=171845;ti(12)=172230;
% car setting run:
% ti(1)=203231;ti(2)=203542;
% ti(3)=203703;ti(4)=204020;
% ti(5)=204200;ti(6)=204447;
% ti(7)=204637;ti(8)=204949;
% ti(9)=205130;ti(10)=205440;
% ti(11)=205620;ti(12)=205940;

ti=Set_time_interval(t,ti,num_of_interval); % change ti to be the index
t=Change_t_to_datetime(t); 
t=Change_t_to_seconds(t,ti);
% backup=[t,file2_hpr,file1_hpr];
% Read Boreas GNSS

[boreas_gnss_t,boreas_gnss_h]=Read_boreas_gnss("23Jan11/jan11_morning_raw_gnss.txt");
boreas_gnss_t=datetime(boreas_gnss_t, 'ConvertFrom', 'posixtime', 'Format', 'HHmmss.SS');
temp=str2double(string(boreas_gnss_t));
boreas_gnss_t=temp;
boreas_gnss_t=Change_t_to_datetime(boreas_gnss_t);
boreas_gnss_t=Change_t_to_seconds(boreas_gnss_t,ti);
boreas_gnss_h_interpolated=interp1(boreas_gnss_t,boreas_gnss_h,t);
% file1_hpr(:,1)=boreas_gnss_h_interpolated;

d_hpr=Find_difference_between_two_instruments(t,file2_hpr,file1_hpr);
%% Find if the maxima have same indices
arr1=file1_hpr(:,3);
find(arr1==max(arr1))
arr2=file2_hpr(:,3);
find(arr2==max(arr2))
%%
plot(boreas_gnss_t,boreas_gnss_h,'--');
%% Plot heading pitch roll
Title=["heading","pitch","roll"];
% t1=max(find(t<-3000));
% t2=min(find(t>-2744));
t1=ti(1);
t2=ti(12);
for i=1:1
    f=figure(i);
    f.Position=[500 200 800 450];
    subplot('Position',[.1 .55 .8 .4]);
    plot(t,file2_hpr(:,i));
    hold on ;
%     plot(t,file1_hpr(:,i),'--');
    plot(boreas_gnss_t,boreas_gnss_h,'--');
    hold off;
    ylabel(Title(i) + " (deg.)","FontSize",11);
%     xlim([t(min(ti)),t(max(ti))]);
%     xlim([t(1),t(max(ti))]);
    xlim([t(t1),t(t2)]);
    for j=1:length(ti) % set xline and its color
        if rem(j,2)==1
            xline(t(ti(j)),'g');
        else
            xline(t(ti(j)),'m');
        end
    end
    legend('ABX-two','Boreas');
    ax = gca;
    ax.FontSize = 12;
    grid on;
    set(gca,'xticklabel',[])
    subplot('Position',[.1 .12 .8 .4]);
    plot(t,d_hpr(:,i),'.','MarkerSize',5);
    xlabel('time (s)',"FontSize",11);
%     xlim([t(min(ti)),t(max(ti))]);
%     xlim([t(1),t(max(ti))]);
%     xlim([t(ts(5)),t(ts(4))]);
    xlim([t(t1),t(t2)]);
    for j=1:length(ti)
        if rem(j,2)==1
            xline(t(ti(j)),'g');
        else
            xline(t(ti(j)),'m');
        end
    end
    legend('difference','start','stop');
    ax = gca;
    ax.FontSize = 12;
    ylabel("Difference (deg.)","FontSize",11);
    grid on;
    if i==1
        ylim([-1,4]);
    end
    
end


%% Plot mean of different runs of same filter setting
file1_mean_pitch=zeros(2,3);
file2_mean_pitch=zeros(2,3);
k=1;
hpr_index=2;
for i=1:3
    for j=1:2
        file1_mean_pitch(j,i)=mean(file1_hpr(ti(k):ti(k+1),hpr_index));
        file2_mean_pitch(j,i)=mean(file2_hpr(ti(k):ti(k+1),hpr_index));
        k=k+2;
    end
end
file1_mean_roll=zeros(2,3);
file2_mean_roll=zeros(2,3);
k=1;
hpr_index=3;
for i=1:3
    for j=1:2
        file1_mean_roll(j,i)=mean(file1_hpr(ti(k):ti(k+1),hpr_index));
        file2_mean_roll(j,i)=mean(file2_hpr(ti(k):ti(k+1),hpr_index));
        k=k+2;
    end
end
file1_mean_pr=cat(1,file1_mean_pitch,file1_mean_roll);
file2_mean_pr=cat(1,file2_mean_pitch,file2_mean_roll);

f=figure(6);
f.Position=[500 200 1600 900];
j=1;
Title=["down pitch","up pitch","down roll","up roll"];
for i=1:4
    subplot(2,2,i);
    plot(file1_mean_pr(j,:),'.-','MarkerSize',10);
    hold on;
    plot(file2_mean_pr(j,:),'.-','MarkerSize',10);
    hold off;
    j=j+1;
    legend("Boreas","ABX");
    title(Title(i));
    xlabel("Run Number");
    ylabel("Mean Value");
end

%% Plot histogram with Gaussian fit
hpr_index=1;
% t1=max(find(t<-3000));
% t2=min(find(t>-2744));
t1=ti(1);
t2=ti(2);
Xlim=[];
Title=["heading","pitch","roll"];
f=figure(4);
f.Position=[500 200 1600 900];
% subplot(3,1,1);
% Plot_histogram(file1_hpr(t1:t2,hpr_index),Title(hpr_index),Xlim,"degree","Boreas");
% subplot(3,1,2);
% Plot_histogram(file2_hpr(t1:t2,hpr_index),Title(hpr_index),Xlim,"degree","ABX-two");
% subplot(3,1,3);
% Plot_histogram(d_hpr(t1:t2,hpr_index),Title(hpr_index)+" difference",[],"degree",[]);

for i=1:3
    subplot(3,1,i)
    Plot_histogram(d_hpr(t1:t2,i),Title(i)+" difference",[],"degree",[]);
end
%% Plot histogram
% t1=max(find(t<-3000));
% t2=min(find(t>-2744));
t1=ti(3);
t2=ti(4);
Title=["Heading","Pitch","Roll"];
figure(4);
histogram(d_hpr(t1:t2,hpr_index));
Legend1 = sprintf('\\mu =%f',mean(d_hpr(t1:t2,hpr_index)));
Legend2 = sprintf('\\sigma =%f',std(d_hpr(t1:t2,hpr_index)));
legend({[Legend1 newline Legend2]},"FontSize",12)
title(Title(hpr_index)+" Residual");
xlabel("degree","FontSize",12);
ax = gca;
ax.FontSize = 12;
xlim([min(d_hpr(t1:t2,hpr_index)),min(d_hpr(t1:t2,hpr_index))+0.4]);
% Plot_histogram(d_hpr(t1:t2,hpr_index),Title(hpr_index)+" difference",[],"degree",[]);
%% Match histogram with time
figure(5);
hpr_index=3;
% t1=1;
% t2=min(find(t>-200));
t1=max(find(t<-3000));
t2=min(find(t>-2744));
% t1=ti(1);
% t2=ti(2);
subplot(2,2,[1,2]);
Offset=mean(file1_hpr(t1:t2,hpr_index)-file2_hpr(t1:t2,hpr_index));
plot(t(t1:t2),file2_hpr(t1:t2,hpr_index)+Offset);
hold on;
plot(t(t1:t2),file1_hpr(t1:t2,hpr_index));
hold off;
legend("ABX","Boreas");
subplot(2,2,3);
plot(t(t1:t2),d_hpr(t1:t2,hpr_index));
title("difference")
subplot(2,2,4);
Plot_histogram(d_hpr(t1:t2,hpr_index),[],[],[],[]);
%% funct Plot histogram
function []=Plot_histogram(y1,Title,Xlim,Xlabel,Ylabel)
histfit(y1);
pd = fitdist(y1,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title(Title);
xlabel(Xlabel);
ylabel(Ylabel);
if isempty(Xlim)==0
    xlim(Xlim);
end
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
%% funct set time interval and change ti to be indices
function ti=Set_time_interval(t,ti,num_of_interval)
% add  1sec to start time and minus 1sec to the end time
% make ti to be the index of the time interval.
for i=1:num_of_interval*2
    if rem(i,2)==1
        ti(i)=ti(i)+1;
    else
        ti(i)=ti(i)-1;
    end
    ti(i)=min(find(t>=ti(i)));
%     if i<=length(ts)
%         ts(i)=min(find(t>=ts(i)));
%     end
end
end


%% funct d_hpr
function d_hpr=Find_difference_between_two_instruments(t,abx_hpr,apx_hpr)
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
%% Read Boreas GNSS
function [boreas_gnss_t,boreas_gnss_h]=Read_boreas_gnss(file3_name)
boreas_gnss=readtable(file3_name);
boreas_gnss_t=boreas_gnss{:,1};
boreas_gnss_h=boreas_gnss{:,5};
temp1=zeros(length(boreas_gnss_t),1);
temp2=zeros(length(boreas_gnss_h),1);
n=0;
for i=2:length(boreas_gnss_t)
    if boreas_gnss_t(i) >= boreas_gnss_t(1) & boreas_gnss_h(i) ~= 0 & boreas_gnss_t(i) ~= boreas_gnss_t(i-1)
        n=n+1;
        temp1(n)=boreas_gnss_t(i);
        temp2(n)=boreas_gnss_h(i);
    end
end
boreas_gnss_t=temp1(1:n);
boreas_gnss_h=temp2(1:n);

end
%% Read in
function [t,file1_hpr,file2_hpr]=Read_in(file1_name,file2_name)
file1 = readtable(file1_name);
file1 = file1{:,:};
file1_time = file1(:,1);
file1_heading = file1(:,5);
file1_pitch = file1(:,6);
file1_roll = file1(:,7);

file2 = readtable(file2_name);
file2_time = file2{:,3};
file2_heading = file2{:,4};
file2_pitch = file2{:,5};
file2_roll = file2{:,6};

file1_time=datetime(file1_time, 'ConvertFrom', 'posixtime', 'Format', 'HHmmss.SS');
temp=str2double(string(file1_time));
file1_time=temp;

N=length(file1_time);

t=zeros(N,1);
file1_h=zeros(N,1);
file1_p=zeros(N,1);
file1_r=zeros(N,1);
file2_h=zeros(N,1);
file2_p=zeros(N,1);
file2_r=zeros(N,1);
n=0;
for i=1:length(file2_time)
    for j=1:length(file1_time)
        if file2_time(i)==file1_time(j)
            if isnan(file2_heading(i)) | isnan(file1_heading(i)) | file1_time(j)==0
                continue
            else
                n=n+1;
                t(n) = file2_time(i);
                file2_h(n) = file2_heading(i);
                file2_p(n) = file2_pitch(i);
                file2_r(n) = file2_roll(i);

                file1_h(n) = file1_heading(j);
                file1_p(n) = file1_pitch(j);
                file1_r(n) = file1_roll(j);
            end
        end
    end
end

t=t(1:n);

file1_hpr=[file1_h,file1_p,file1_r];
file1_hpr=file1_hpr(1:n,:);
file2_hpr=[file2_h,file2_p,file2_r];
file2_hpr=file2_hpr(1:n,:);
end
