% num =
% 
%          0    7.8387   -7.8387
% 
% 
% den =
% 
%     1.0000   -1.5622    0.6413
    
    
    
    
    
    close all;
    
%Hd = testFilt();

t = 0:0.01:10;

omega = 2*pi*1;
A = 10;
u = A*cos(omega*t)-1 + normrnd(0,0.5,1,length(t));

y_an = -A*omega*sin(omega*t) ;

%y = filter(Hd,u)*100;
omega_c = 2*pi*2;
delta = 1/sqrt(2)*1.5;
Hct = tf([omega_c.^2 0],[1 2*delta*omega_c omega_c.^2]);
Hdt = c2d(Hct,0.01);
figure(1)
bode(Hct,Hdt);

y_man = lsim(Hdt,u,t);

figure(2)
plot(t,u, ...
    t,y_an, ...
    ...%t,y, ...
    t,y_man)
legend("u", ...
    "y analytical", ...
    ...%"y", ...
    "y from CT")

v = zeros(size(u));
[num,den] = tfdata(Hdt,'v');

for i = 3:length(t)
    v(:,i) = num(1)*u(:,i) + num(2)*u(:,i-1) + num(3)*u(:,i-2) -  den(2)*v(:,i-1) - den(3)*v(:,i-2); 
end

%%
figure()
plot(t,v);hold on;
plot(t,y_an);
plot(t,y_man);
legend('manual','analytical','one shot')














