[file1_th,file2_th]=Read_in("23Jan18/jan18_afternoon_system_state.txt","23Jan18/jan18_afternoon_raw_gnss.txt");
t=file1_th(:,1);
num_of_interval=1;
ti=zeros(num_of_interval*2,1);
% mourning
ti(1)=163709; ti(2)=172300;
% afternoon
% ti(1)=185508; ti(2)=205912;
ti=Set_time_interval(t,ti,num_of_interval);
file1_t=Change_t_to_datetime(file1_th(:,1));
file1_t=Change_t_to_seconds(file1_t,ti);
file2_t=Change_t_to_datetime(file2_th(:,1));
file2_t=Change_t_to_seconds(file2_t,ti);
t=Change_t_to_datetime(t);
t=Change_t_to_seconds(t,ti);
%%
std(file1_th(42408:end,2))
%%
valid_index=min(find(file1_progress==100));

%%
f=figure(1);
f.Position=[500 200 1600 900];
t1=193;t2=7000;
plot(file1_t,file1_th(:,2));
% xline(t1,'g');
% xline(t2,'m');
% xline(3068.5,'g');
% xline(3529.4,'m');
hold on;
plot(file2_t,file2_th(:,2),'--');
hold off;
legend("Gyrocompass","GNSS");
xlabel('time (s)',"FontSize",11);
ylabel('heading (degree)',"FontSize",11);
% xlim([t1,t2])
grid on;
ax = gca;
ax.FontSize = 12;
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
%%
function [file1_th,file2_th]=Read_in(file1_name,file2_name)
file1 = readtable(file1_name);
file1 = file1{:,:};
file1_time = file1(:,1);
file1_heading = file1(:,5);
% file1_progress = file1(:,9);
file2 = readtable(file2_name);
file2 = file2{:,:};
file2_time = file2(:,1);
file2_heading = file2(:,5);


n=0;
file1_t=zeros(length(file1_time),1);
file1_h=zeros(length(file1_time),1);
for i=1:length(file1_time)
    if isnan(file1_time(i)) | file1_time(i)==0 | isnan(file1_heading(i)) | file1_heading(i)==0
        continue
    else
        n=n+1;
        file1_t(n)=file1_time(i);
        file1_h(n)=file1_heading(i);
    end
end
temp=file1_t(1:n);
file1_t=temp;
temp=file1_h(1:n);
file1_h=temp;

n=0;
file2_t=zeros(length(file2_time),1);
file2_h=zeros(length(file2_heading),1);
for i=1:length(file2_time)
    if isnan(file2_time(i)) | file2_time(i)==0 | isnan(file2_heading(i)) | file2_heading(i)==0
        continue
    else
        n=n+1;
        file2_t(n)=file2_time(i);
        file2_h(n)=file2_heading(i);
    end
end
temp=file2_t(1:n);
file2_t=temp;
temp=file2_h(1:n);
file2_h=temp;

file1_t=datetime(file1_t, 'ConvertFrom', 'posixtime', 'Format', 'HHmmss.SS');
temp=zeros(length(file1_t),1);
for i=1:length(file1_t)
    temp(i)=str2num(string(file1_t(i)));
end
file1_t=temp;

temp=zeros(length(file2_t),1);
file2_t=datetime(file2_t, 'ConvertFrom', 'posixtime', 'Format', 'HHmmss.SS');
temp=zeros(length(file2_t),1);
for i=1:length(file2_t)
    temp(i)=str2num(string(file2_t(i)));
end
file2_t=temp;
% extract points with same time stamp
% N=0;
% for i=1:length(file2_time)
%     for j=1:length(file1_time)
%         if file2_time(i)==file1_time(j)
%             if isnan(file2_heading(i)) | isnan(file1_heading(i))
%                 continue
%             else
%                 N=N+1;
%             end
%         end
%     end
% end
% t=zeros(N,1);
% file1_h=zeros(N,1);
% file1_progress=zeros(N,1);
% file2_h=zeros(N,1);
% 
% n=0;
% for i=1:length(file2_time)
%     for j=1:length(file1_time)
%         if file2_time(i)==file1_time(j)
%             if isnan(file2_heading(i)) | isnan(file1_heading(i))
%                 continue
%             else
%                 n=n+1;
%                 t(n) = file2_time(i);
%                 file2_h(n) = file2_heading(i);
%                 file1_progress(n)=file1_prog(j)
%                 file1_h(n) = file1_heading(j);
%             end
%         end
%     end
% end

file1_th=[file1_t,file1_h];
file2_th=[file2_t,file2_h];
end
