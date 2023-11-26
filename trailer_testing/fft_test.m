a=zeros(97,1);
for i=1:length(a)
    if rem(i,2)==1
        a(i)=2.5;
    else
        a(i)=1.5;
    end
end
%%
plot(abs(fft(a)))
%%
b=1:10000;
b=b/10000;
fs=(0:length(b)-1) ;
c=sin(2*pi*b*5);
subplot(2,1,1);
plot(b,c);
subplot(2,1,2);
fftc=abs(fft(c))/length(b);
plot(fs,fftc);
% fftc(11)=1;
% fftc(11)
% plot(b,ifft(fftc),'.');
%%
0:4
%%
subplot(2,1,1);
plot(a)
subplot(2,1,2);
plot(b,fft(a));