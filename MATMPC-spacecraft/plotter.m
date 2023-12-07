clear all;
clc;
close all;
%nexttile(4,[2,1])
s1 = load('state_b');
s2 = load('state_lqr');
s3 = load('state_2.mat');
s4 = load('state_3.mat');
s5 = load('state_base.mat');
inp  = load('input_obs.mat');
t = load('time')';
s6 = load('state_obs.mat')

controls_MPC = inp.controls_MPC;
inpod = [1,1,0];
inpod = inpod/norm(inpod);
time = t.time;
state_sim_base = s1.state_sim;
state_sim2  = s2.state_sim;
state_sim3  = s3.state_sim;
state_sim4  = s4.state_sim;
state_sim5  = s5.state_sim;
state_sim6  = s6.state_sim;


figure()
set(gca,'fontsize',14)
plot(time,state_sim_base(:,1:3),'LineWidth',2);
grid on
ylabel('$p$ [m]','interpreter','latex','fontweight','bold','fontsize',14);
xlabel('$t$ [s]','interpreter','latex','fontweight','bold','fontsize',14);

legend('$p_x$','$p_y$','$p_z$','interpreter','latex','fontweight','bold','fontsize',14)

figure()
set(gca,'fontsize',14)
% distance
plot(time,vecnorm(state_sim_base(:,1:3),2,2),'LineWidth',2);
grid on
ylabel('$d$ [m]','interpreter','latex','fontweight','bold','fontsize',14);
xlabel('$t$ [s]','interpreter','latex','fontweight','bold','fontsize',14)


figure()  % linear vel
set(gca,'fontsize',14)

plot(time,state_sim_base(:,4:6),'LineWidth',2);
grid on
ylabel('$v$ [m/s]','interpreter','latex','fontweight','bold','fontsize',14);
xlabel('$t$ [s]','interpreter','latex','fontweight','bold','fontsize',14)
legend('$v_x$','$v_y$','$v_z$','interpreter','latex','fontweight','bold','fontsize',14)

figure()  % controls
set(gca,'fontsize',14)

plot(time,controls_MPC,'LineWidth',2);
grid on
ylabel('$command$','interpreter','latex','fontweight','bold','fontsize',14);
xlabel('$t$ [s]','interpreter','latex','fontweight','bold','fontsize',14)
%legend('$a_1$','$a_2$','$a_3$')

figure()
set(gca,'fontsize',14)

% plot3(state_sim_base(:,1),state_sim_base(:,2),state_sim_base(:,3),'b','Linewidth',2)
plot3(state_sim2(:,1),state_sim2(:,2),state_sim2(:,3),'g','Linewidth',2);
hold on
plot3(state_sim_base(:,1),state_sim_base(:,2),state_sim_base(:,3),'b','Linewidth',2)

plot3(state_sim4(:,1),state_sim4(:,2),state_sim4(:,3),'k','Linewidth',2)
plot3(state_sim6(:,1),state_sim6(:,2),state_sim6(:,3),'y','Linewidth',2)


plot3(10*[0 inpod(1)],10*[0 inpod(2)],10*[0 inpod(3)],'r')
plot3(state_sim_base(1,1),state_sim_base(1,2),state_sim_base(1,3),'*b','Linewidth',2)
plot3(state_sim_base(end,1),state_sim_base(end,2),state_sim_base(end,3),'ob','Linewidth',2)
plot3(state_sim4(1,1),state_sim4(1,2),state_sim4(1,3),'*k','Linewidth',2)      
plot3(state_sim4(end,1),state_sim4(end,2),state_sim4(end,3),'ok','Linewidth',2)
plot3(state_sim2(1,1),state_sim2(1,2),state_sim2(1,3),'*g','Linewidth',2)
plot3(state_sim2(end,1),state_sim2(end,2),state_sim2(end,3),'og','Linewidth',2)
plot3(state_sim6(1,1),state_sim6(1,2),state_sim6(1,3),'*y','Linewidth',2)
plot3(state_sim6(end,1),state_sim6(end,2),state_sim6(end,3),'oy','Linewidth',2)
[X,Y,Z] = ellipsoid(0,0,0,10,10,10);
surf(X,Y,Z,'EdgeColor','none','FaceAlpha',0.1,'FaceColor','m')
legend('LQR','MPC1','MPC2','MPC Obs','sun')



plot3(0,0,0,'*r','Linewidth',2)

axis equal
grid on
xlabel('$x$ [m]','interpreter','latex','fontweight','bold','fontsize',14)
ylabel('$y$ [m]','interpreter','latex','fontweight','bold','fontsize',14)
zlabel('$z$ [m]','interpreter','latex','fontweight','bold','fontsize',14)
view([37 30])
      
