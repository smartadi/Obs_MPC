clc;
clear all;
close all;


n= 4;
m= 2;
l= 2;
N= 500;

A = [0,0,1,0;
     0,0,0,1;
     0,0,0,0;
     0,0,0,0];

B = [0 0;0 0;1 0;0 1];



C = [1,0,0,0;
     0,1,0,0];
 
D = [];

dt = 0.01;

sys1 = ss(A,B,C,D);
sys2 = c2d(sys1,dt);

At = sys2.A;
Bt = sys2.B;
Ct = sys2.C;

Qx = 1*eye(2);
Qv = 0.1*eye(2);
Q = [Qx,zeros(2,2);
    zeros(2,2),Qv];
R = 0.01*eye(2);

[K ,S,e] = dlqr(At,Bt,Q,R,[]); 



t = N;
X0 = [1,1,1,0]'; 
T = eye(t);
m = 2;
n = 4 ;

QQ = kron(T,Q);
RR = kron(T,R);
AA = [];

for i = 1:t
    AA = [AA;At^(i)];
end
H=[];

for i = 1:t
    CC = [];
    for j = 1:t
        if j<=i
            c = At^(i-j+1)*Bt;
        else
            c = zeros(n,m);
        end
        CC = [CC,c];
    end
        H = [H;CC];
end
%%
    cvx_begin 
        variable U(m*t)
        minimize ((AA*X0 + H*U)'*QQ*(AA*X0 + H*U) + U'*RR*U) 
    cvx_end
%%

 X = AA*X0 + H*U;
 x = reshape(X,4,t);
 
 %%
figure()
plot(x(1,:),x(2,:))
title('LQR cvx')

%%
 
xx(:,1) = X0;
c=0;
for i=1:N
    u(:,i) = -K*xx(:,i);
    xx(:,i+1) = At*xx(:,i) + Bt*u(:,i);
    y(:,i) = Ct*xx(:,i);
    c = c + xx(:,i)'*Q*xx(:,i) + u(:,i)'*R*u(:,i);
end


figure()
plot(xx(1,:),xx(2,:));
title('Lqr solved')

%% Modified 

figure()
plot(xx(1,:),xx(2,:),'Linewidth',2);hold on
for s = 1:10
xxx = zeros(size(xx));    
xxx = X0 + normrnd(0,0.3,4,1);
c=0;
for i=1:N
    
    xxx(:,i+1) = At*xxx(:,i) + Bt*u(:,i);
    y(:,i) = Ct*xxx(:,i);

end

plot(xxx(1,:),xxx(2,:));
end
plot(X0(1),X0(2),'go','Linewidth',3)
plot(xx(1,end),xx(2,end),'ro','Linewidth',3)
legend('orig','shift')
title('Lqr solved')
%%
% r1=.1;
% r2=.2;
% xc=1;
% yc=1;
% theta = pi/6;
% t = linspace(0, 2*pi, 200);
% xt = r1 * cos(t) + xc;
% yt = r2 * sin(t) + yc;
% % aply rotation by angle theta
% cot = cos(theta); sit = sin(theta);
% x = xt * cot - yt * sit;
% y = xt * sit - yt * cot;
% % draw the curbe
% plot(x, y, '-');