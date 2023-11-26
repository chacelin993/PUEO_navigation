%% Read in
abx = readtable("August11/abx_two.txt");
apx = readtable("August11/apx_18.txt");
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
            if isnan(abx_heading(i))==0 && isnan(apx_heading(i))==0
                t(end+1) = abx_attitude_time(i);
                abx_h(end+1) = abx_heading(i);
                abx_p(end+1) = abx_pitch(i);
                abx_r(end+1) = abx_roll(i);
    
                apx_h(end+1) = apx_heading(j);
                apx_p(end+1) = apx_pitch(j);
                apx_r(end+1) = apx_roll(j);
            end
            break;
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

% find dR of heading pitch roll
% after doing this dh/p/r_range will plot rotation matrices difference
% instead of just the difference.
d_heading=ones(length(t),1);
d_pitch=ones(length(t),1);
d_roll=ones(length(t),1);

for i=1:length(t)
    R_b = R(abx_h(i),abx_p(i),abx_r(i));
    R_p = R(apx_h(i),apx_p(i),apx_r(i));
    dR = R_b*R_p.';
    temp=get_angle(dR);
    d_heading(i) = temp(1);
    d_pitch(i) = temp(2);
    d_roll(i) = temp(3);
end

% for i=1:length(d_heading)
%     if d_heading(i)>350
%         d_heading(i) = d_heading(i)-360 ;
%     end
%     if d_heading(i)<-350
%         d_heading(i) = d_heading(i)+360 ;
%     end
% end
%% Heading
figure(1);
subplot(2,1,1);
plot(t,abx_h,t,apx_h,'--');
title('heading');
legend('abx','apx');
xline(160325,':','fast','HandleVisibility','off');
xline(160910,':','slow','HandleVisibility','off');
xline(161230,':','stop~','HandleVisibility','off');
xline(161635,':','~stop','HandleVisibility','off');
subplot(2,1,2);
plot(t,abx_h-apx_h,'.','MarkerSize',3);
ylabel('abx-apx')
xlabel('time');
xline(160325,':','fast');
xline(160910,':','slow');
xline(161230,':','stop~');
xline(161635,':','~stop');
ylim([-2,5]);
%% Pitch
figure(2)
subplot(2,1,1);
plot(t,abx_p,t,apx_p,'--');
title('pitch');
legend('abx','apx');
xline(160325,':','fast','HandleVisibility','off');
xline(160910,':','slow','HandleVisibility','off');
xline(161230,':','stop~','HandleVisibility','off');
xline(161635,':','~stop','HandleVisibility','off');
subplot(2,1,2);
plot(t,abx_p-apx_p,'.','MarkerSize',3);
ylabel('abx-apx');
xlabel('time');
xline(160325,':','fast');
xline(160910,':','slow');
xline(161230,':','stop~');
xline(161635,':','~stop');
%% Roll
figure(3);
subplot(2,1,1);
plot(t,abx_r,t,apx_r,'--');
title('roll');
legend('abx','apx');
xline(160325,':','fast','HandleVisibility','off');
xline(160910,':','slow','HandleVisibility','off');
xline(161230,':','stop~','HandleVisibility','off');
xline(161635,':','~stop','HandleVisibility','off');
subplot(2,1,2);
plot(t,abx_r-apx_r,'.','MarkerSize',3);
ylabel('abx-apx');
xlabel('time');
xline(160325,':','fast');
xline(160910,':','slow');
xline(161230,':','stop~');
xline(161635,':','~stop');


%% dh_range
%first range
t1=190540; t2=191229-2;
t3=184007; t4=184125;
t5=184418; t6=184842;
t7=185244; t8=185702;
t9=185950; t10=190058;
title1='fast'; title2='slow';
title3='fast'; title4='slow';
figure(5)
subplot(3,2,1)
dh_range1=d_heading(min(find(t>t1)):max(find(t<t2)));
histfit(dh_range1);
pd = fitdist(dh_range1,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title(title1);
subplot(3,2,3)
%fast
dh_range2=d_heading(min(find(t>t3)):max(find(t<t4)));
for i=1:length(dh_range2)
    if dh_range2(i)>350
        dh_range2(i) = dh_range2(i)-360 ;
    end
end
histfit(dh_range2);
pd = fitdist(dh_range2,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title(title2);
subplot(3,2,4);
%slow
dh_range3=d_heading(min(find(t>t5)):max(find(t<t6)));
for i=1:length(dh_range3)
    if dh_range3(i)<-350
        dh_range3(i) = dh_range3(i)+360 ;
    end
end
histfit(dh_range3);
title(title3);
pd = fitdist(dh_range3,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
%stop
subplot(3,2,5);
dh_range4=d_heading(min(find(t>t7)):max(find(t<t8)));
histfit(dh_range4);
pd = fitdist(dh_range4,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title(title4);
subplot(3,2,6);
dh_range5=d_heading(min(find(t>t9)):max(find(t<t10)));
histfit(dh_range5);
pd = fitdist(dh_range5,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
%% dp_range
%norm
figure(5);
subplot(2,1,1);
dp_range1=d_pitch(min(find(t>t1)):max(find(t<t2)));
histfit(dp_range1);
pd = fitdist(dp_range1,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title('first range');
%fast
subplot(2,1,2);
dp_range2=d_pitch(min(find(t>t3)):max(find(t<t4)));
histfit(dp_range2);
pd = fitdist(dp_range2,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title('second range');
%% dr_range
figure(6)
subplot(2,1,1);
dr_range1=d_roll(min(find(t>t1)):max(find(t<t2)));
histfit(dr_range1);
pd = fitdist(dr_range1,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title('first range');
%fast
subplot(2,1,2);
dr_range2=d_roll(min(find(t>t3)):max(find(t<t4)));
histfit(dr_range2);
pd = fitdist(dr_range2,'Normal');
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
title('second range');

%%
r_dh = zeros(length(t),1);
r_dp = zeros(length(t),1);
r_dr = zeros(length(t),1);
for i=1:length(t)
    R_b = R(abx_h(i),abx_p(i),abx_r(i));
    R_p = R(apx_h(i),apx_p(i),apx_r(i));
    dR = R_b*R_p.';
    temp=get_angle(dR);
    r_dh(i)=temp(1);
    r_dp(i)=temp(2);
    r_dr(i)=temp(3);
end
figure(8);
subplot(3,1,1);
histfit(r_dh);
pd = fitdist(r_dh,"Normal");
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
subplot(3,1,2);
histfit(r_dp);
pd = fitdist(r_dp,"Normal");
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
subplot(3,1,3)
histfit(r_dr);
pd = fitdist(r_dr,"Normal");
Legend = sprintf('\\mu =%f \\sigma =%f',pd.mu,pd.sigma);
legend('',Legend);
%%
mean(dp_range1)
mean(dp_range2)
%%
R1=R(30,35,40);
R2=R(5,5,5);
R3=R(35,40,45);

get_angle(R3);

dR=R3*R1.';
get_angle(dR)
dR=R1*R3.';
get_angle(dR)
dR*R1;
R3;
%%
e1=[1;0;0];
R(0,90,0)*e1