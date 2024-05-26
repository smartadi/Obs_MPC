clc;
close all;
clear all;
x = 4*(0.5-rand(2,10000));


norm(x(:,1))
norm(x(:,10))

r=2;
Q = r^2*[1 0;
    0 1];

A = 0.9*rand(2,2);

A = [0.9 0;
    0 0.8];

eigs(A)
T = A^10;
xx = T*x;

figure()
plot(x(1,:),x(2,:),'bo');hold on;
plot(xx(1,:),xx(2,:),'ro');


R = T'*Q*T;
R2 = inv(T)'*inv(Q)*inv(T);
R3 = inv(R);
eigs(R);

eigs(R2);

z=[];
w=[];
for i=1:1000
    y = x(:,i);
    yy = xx(:,i);
    if y'*inv(Q)*y <= 1
        z = [z,y];
    end
    
    if yy'*R3*yy <= 1
        w = [w,yy];
    end
end


figure()
plot(z(1,:),z(2,:),'bo');hold on;
plot(w(1,:),w(2,:),'ro');