clear all;
close all;
clc;
% dt = 1/35;
dt = 0.0285;
t = 0:dt:dt*1000;

path = "/run/user/1000/gvfs/smb-share:server=steinmetzsuper1.biostr.washington.edu,share=data/Subjects/ZYE_0069/2023-10-03/1";
upath = '/corr/svdSpatialComponents_ortho.npy';
upath = append(path,upath);
vpath = '/corr/svdTemporalComponents_ortho.npy';
vpath = append(path,vpath);


Uu = readUfromNPY(upath);
Vv = readVfromNPY(vpath);

[TT,dims]= size(Vv);

Vv = Vv./vecnorm(Vv,2,2);

disp('loading WF PCA projections')

data1=Vv';
N=10000;
%%
n = 5;
l = n;
Ts = 1000;
data_small = data1(2:l,1:Ts);
nn = 8;

%   We will now identify this system from the data y 
%   with the subspace identification algorithm: subid
%   
%   The only extra information we need is the "number of block rows" i
%   in the block Hankel matrices.  This number is easily determined
%   as follows:

%   Say we don't know the order, but think it is maximally equal to 10.

       max_order = 10;
%   
%   As described in the help of subid we can determine "i" as follows:
%   
%       i = 2*(max_order)/(number of outputs)
%       must be an integer
       p = 2*(max_order)/l;

  % nn= 10;

 AUX=[];

 [A,du1,C,du2,K,R,AUX] = subid(data_small,[],p,nn,[],[],1);

 [As,du1s,Cs,du2s,Ks,Rs] = subid_stable(data_small,[],p,nn,AUX,'sv');


%%
dd = eig(A)
ds = eig(As)
figure()
plot(dd,'ob');hold on
plot(ds,'or');
legend('unstable','stable')




%%

Q = TT/Ts;
% Q = 5.5;
Vd = zeros(nn,nn,floor(Q));
Ds=[];
D=[];
for i = 1:1:Q
    % data = data1(1:l,Ts*(i-1)+1 : i*Ts);
    data = data1(2:l,Ts*(i-1)+1 : i*Ts);

    AUX=[];
    [A,du1,C,du2,K,R,AUX] = subid(data,[],p,nn,[],[],1);

    [As,du1s,Cs,du2s,Ks,Rs] = subid_stable(data,[],p,nn,AUX,'sv');

    %As = As - Ks*Cs;

    [E,V] = eig(As)
    
    Vd(:,:,i) = V;

    D = [D,eig(A)];
    Ds = [Ds,eig(As)];
eig(A)
end
%%
% close all;
% figure()
% plot(Ds,'or'); hold on;
% axis([-1 1 -1 1])
% title('stable eigenvalues')
% cDs = Ds/dt;
% cD = D/dt;
% 
% t= 1:1:Q;
% 
% figure()
% plot(t,abs(imag(cDs)),'or'); hold on;
% title('Stable continuous time eigenvalues')
% 
% figure()
% plot(t,abs(imag(cD)),'or'); hold on;
% title('Stable continuous')
close all;
figure()
plot(Ds,'or'); hold on;
axis([-1 1 -1 1])
title('stable eigenvalues')


figure()
plot(D,'or'); hold on;
axis([-1 1 -1 1])
title('unstable eigenvalues')
cDs = (Ds-1)/dt;
cD = D/dt;

t= 1:1:Q;

figure()
plot(t,abs(imag(cDs)),'or'); hold on;
title('Stable continuous time eigenvalues')

figure()
plot(t,abs(imag(cD)),'ob'); hold on;
title('unstable continuous')

%% power spectrum
% close all;
% [p,f] = 

L=10000;
S = Vv(1:L,1:10);

Fs = 35
% 
% f = Fs/L*(0:(L-1));
% 
% figure()
% plot(f,S,"LineWidth",3) 
% title("Single-Sided Amplitude Spectrum of X(t)")
% xlabel("f (Hz)")
% ylabel("|P1(f)|")
% 
% 
% Y = fft(S);
% P2 = abs(Y/L);
% P1 = P2(1:L-1);
% P1(2:end-1) = 2*P1(2:end-1);
% 
% figure()
% plot(f,Y,"LineWidth",3) 
% title("Single-Sided Amplitude Spectrum of S(t)")
% xlabel("f (Hz)")
% ylabel("|P1(f)|")
% 
% 
% 
% plot(Fs/L*(-L/2:L/2-1),abs(fftshift(Y)),"LineWidth",3)
% title("fft Spectrum in the Positive and Negative Frequencies")
% xlabel("f (Hz)")
% ylabel("|fft(X)|")
% 
% 
% power = abs(Y).^2/n;    % power of the DFT
% 
% figure()
% plot(f,power)
% xlabel('Frequency')
% ylabel('Power')
%%

% [pxx,f] = pspectrum(abs(Y),35);
% 
% figure()
% plot(f,pxx)
% xlabel('Frequency (Hz)')
% ylabel('Power Spectrum (dB)')
% title('Default Frequency Resolution')

%%

% Fs = 35;            % Sampling frequency                    
% T = 1/Fs;             % Sampling period       
% L = 10000;             % Length of signal
% t = (0:L-1)*T;        % Time vector
% 
% 
% Y = fft(S);

% figure()
% plot(Fs/L*(0:L-1),abs(Y),"LineWidth",3)
% title("Complex Magnitude of fft Spectrum")
% xlabel("f (Hz)")
% ylabel("|fft(X)|")
% 
% figure()
% plot(Fs/L*(-L/2:L/2-1),abs(fftshift(Y)),"LineWidth",3)
% title("fft Spectrum in the Positive and Negative Frequencies")
% xlabel("f (Hz)")
% ylabel("|fft(X)|")

% a = abs(fftshift(Y));
% tt= Fs/L*(0:L/2-1); 
% figure()
% plot(tt,a(L/2+1:end,:),"LineWidth",3)
% title("fft Spectrum in the Positive and Negative Frequencies")
% xlabel("f (Hz)")
% ylabel("|fft(X)|")
% 
% figure()
% plot(tt,a(L/2+1:end,1),"LineWidth",3)
% title("fft Spectrum in the Positive and Negative Frequencies")
% xlabel("f (Hz)")
% ylabel("|fft(X)|")
% 
% figure()
% plot(tt,a(L/2+1:end,2:end),"LineWidth",3)
% title("fft Spectrum in the Positive and Negative Frequencies")
% xlabel("f (Hz)")
% ylabel("|fft(X)|")

%%

