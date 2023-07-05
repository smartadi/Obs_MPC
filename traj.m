clc;
close all;
clear all;
% mu = 3.986e14;
% a = 2e6;
% n = sqrt(mu/a^3)



Radius_Earth = 6.37814e+06; % [m]
Altitude_Ego_SC = 500e3; % [m]
InterSC_distance_init = 20; % [m]
 mu = 3.986e+14;
 a = Radius_Earth + Altitude_Ego_SC;
 n = sqrt(mu/a^3);
 
A = [0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1;
    3*n^2,0,0, 0, 2*n, 0;
    0, 0, 0, -2*n, 0, 0;
    0, 0, -n^2, 0, 0, 0];


B = [0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    1, 0, 0;
    0, 1, 0;
    0, 0, 1];

%x0 = [0,0,30,0,0,0]';
x01 = 10;
y01 = 20;
z01 = 25;
vx01 = n*y01/2;
vy01 = 0;
vz01 = n*y01/2;


x0 = [x01,y01,z01,vx01,vy01,vz01]';

n = 6;
m = 3;
l = 3;
T = 5000;
dt = 1;

x = zeros(n,T);
u = zeros(m,T);
y = zeros(l,T);

x(:,1) = x0;


Q = eye(6);
R = eye(3);
L = eye(3);


s = [1;0;0];

C = [eye(3),zeros(3,3)];
cost = [];
c=0;
for i =1:T
    x(:,i+1) = x(:,i) + dt*(A*x(:,i) + B*u(:,i));
    y(:,i) = phi(x(1:3,i+1),dot(x(1:3,i)/vecnorm(x(1:3,i)),s));
    
    cost = [cost,c + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i) + (y(:,i) - C*x(:,i))'*L*(y(:,i) - C*x(:,i))];
end

figure()
plot(x(1,:));hold on;
plot(x(2,:));
plot(x(3,:));


figure()
plot(x(4,:));hold on;
plot(x(5,:));
plot(x(6,:));

figure()
plot3(x(1,:),x(2,:),x(3,:));
xlabel('x')
ylabel('y')
zlabel('z')

figure()
plot3(x(1,:),x(2,:),x(3,:));hold on;
plot3(y(1,:),y(2,:),y(3,:));
xlabel('x')
ylabel('y')
zlabel('z')

figure()
plot(cost)


function y = phi(x,s)   
    y = x + norm(1.5-s)*2*(0.5-rand(size(x)));
end