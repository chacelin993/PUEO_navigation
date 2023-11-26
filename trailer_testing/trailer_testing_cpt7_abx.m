cpt7_thpr=Read_CPT7("23April14/cpt7.txt");
abx_thpr=Read_ABX_Two("23April14/abx-two.txt");
% abx_thpr=Read_Filtered_ABX_Two("23March28/abx_filtered.txt");
[t_utc,cpt7_hpr,abx_hpr]=Align_time(cpt7_thpr,abx_thpr); % Align the time scale
d_hpr=Find_difference_between_two_instruments(abx_hpr,cpt7_hpr);
%%
num_of_interval=9;
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

%%
Plot_hpr_with_time(t,ti,cpt7_hpr,abx_hpr,d_hpr);
%% Plot histogram with Gaussian fit
% t1=max(find(t<-3000));
% t2=min(find(t>-2744));

t1=ti(1);
t2=ti(2);
Xlim=[];
Title=["heading","pitch","roll"];
f=figure(4);
f.Position=[500 200 800 450];
hpr_index=1;
subplot(3,1,1);
Plot_histogram(cpt7_hpr(t1:t2,hpr_index),Title(hpr_index),Xlim,"degree","CPT-7");
subplot(3,1,2);
Plot_histogram(abx_hpr(t1:t2,hpr_index),Title(hpr_index),Xlim,"degree","ABX-two");
subplot(3,1,3);
Plot_histogram(d_hpr(t1:t2,hpr_index),Title(hpr_index)+" difference",[],"degree",[]);
% for i=1:3
%     subplot(3,1,i)
%     Plot_histogram(d_hpr(t1:t2,i),Title(i)+" difference",[],"degree",[]);
% end

% j=1;
% run_num=1;
% while j<length(ti)-1
%     t1=ti(j);
%     t2=ti(j+1);
%     for i=1:3
%     subplot(3,1,i)
%     Plot_histogram(d_hpr(t1:t2,i),Title(i)+" difference",[],"degree",[]);
%     end
%     saveas(gcf, "23April14/land_2hist_down"+run_num+".png");
%     j=j+2;
%     t1=ti(j);
%     t2=ti(j+1);
%     for i=1:3
%     subplot(3,1,i)
%     Plot_histogram(d_hpr(t1:t2,i),Title(i)+" difference",[],"degree",[]);
%     end
%     saveas(gcf, "23April14/land_2hist_up"+run_num+".png");
%     run_num=run_num+1;
%     j=j+2;
% end
%%
a="sdf";
a+1
%% funct plot heading pitch roll
function Plot_hpr_with_time(t,ti,file1_hpr,file2_hpr,d_hpr)
Title=["heading","pitch","roll"];
% t1=max(find(t<-3000));
% t2=min(find(t>-2744));
t1=ti(17);
t2=ti(18);
for i=1:3
    f=figure(i);
    f.Position=[500 200 1600 900];
    subplot('Position',[.1 .55 .8 .4]);
    plot(t,file2_hpr(:,i));
    hold on ;
    plot(t,file1_hpr(:,i),'--');
%     plot(boreas_gnss_t,boreas_gnss_h,'--');
    hold off;
    ylabel(Title(i) + " (deg.)","FontSize",14);
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
    legend('ABX-two','CPT-7');
    ax = gca;
    ax.FontSize = 12;
    grid on;
    set(gca,'xticklabel',[])
    subplot('Position',[.1 .12 .8 .4]);
    plot(t,d_hpr(:,i),'.','MarkerSize',5);
    xlabel('time (s)',"FontSize",14);
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
    ylabel("Difference (deg.)","FontSize",14);
    grid on;
%     if i==1
%         ylim([-1,4]);
%     end
end
end
%% funct Plot histogram
function []=Plot_histogram(y1,Title,Xlim,Xlabel,Ylabel)
histfit(y1);
pd = fitdist(y1,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend,"FontSize",18);
title(Title,"FontSize",14);
xlabel(Xlabel,"FontSize",14);
ylabel(Ylabel,"FontSize",14);
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
function d_hpr=Find_difference_between_two_instruments(abx_hpr,apx_hpr)
d_heading=ones(length(abx_hpr),1);
d_pitch=ones(length(abx_hpr),1);
d_roll=ones(length(abx_hpr),1);
abx_h=abx_hpr(:,1);abx_p=abx_hpr(:,2);abx_r=abx_hpr(:,3);
apx_h=apx_hpr(:,1);apx_p=apx_hpr(:,2);apx_r=apx_hpr(:,3);
for i=1:length(d_heading)
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
%% Funct Align_time
function [t,file1_hpr,file2_hpr]=Align_time(file1,file2)
if length(file1)>=length(file2)
    N=length(file1);
else
    N=length(file2);
end
t=zeros(N,1);
file1_hpr=zeros(N,3);
file2_hpr=zeros(N,3);
n=0;
for i=1:length(file2)
    for j=1:length(file1)
        if file2(i,1)==file1(j,1)
            if file1(j,1)==0 | file2(i,1)==0
                continue
            else
                n=n+1;
                t(n) = file2(i,1);
                file2_hpr(n,:) = file2(i,2:4);
                file1_hpr(n,:) = file1(j,2:4);
            end
        end
    end
end
t=t(1:n);
file1_hpr=file1_hpr(1:n,:);
file2_hpr=file2_hpr(1:n,:);
end
%% funct read filtered ABX
function [file]=Read_Filtered_ABX_Two(file_name)
file = readtable(file_name);
file=file{:,:};
file(:,1)=str2double(string(datetime(file(:,1), 'ConvertFrom', 'posixtime', 'Format', 'HHmmss.SS')-hours(4)));

end
%% Funct Read ABX
function [file1]=Read_ABX_Two(file_name)
file = readtable(file_name,'Delimiter', ',');
N=height(file);
file1=zeros(N,4);
n=0;
for i=1:N
    if string(file{i,1}) == '$PASHR' & file{i,4}>=0.1 
        n=n+1;
        file1(n,:)=[file{i,3},file{i,4},file{i,5},file{i,6}];
    end
end
file1=file1(1:n,:);
end

%% Funct Read CPT7
function [file1]=Read_CPT7(file_name)
file = readtable(file_name,'Delimiter', ',');
N=height(file);
file1=zeros(N,4);
n=0;
for i=1:N
    if string(file{i,1}) == '#INSATTA' & file{i,4}>=0.1 
        n=n+1;
        temp=datetime('1980-01-06 00:00:00') + seconds(file{i,6}*7*3600*24 + file{i,7}-18);
        temp=str2double(string(datetime(temp,'Format','HHmmss.SS')));
        file1(n,:)=[temp,file{i,14},file{i,13},file{i,12}];
    end
end
file1=file1(1:n,:);
end