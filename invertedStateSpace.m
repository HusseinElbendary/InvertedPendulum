close all
AnimationEnable=0;
PendulumFrictionEnable=1;
%system parameters
mc=0.75;
mp=0.5;
l=0.6;L=0.3;
I=0.01;
bc=0.1;
bp=0.005;
if PendulumFrictionEnable==0
    bp=0;
end
g=9.81;

%system linearized dynamics around x3=0
D=I*(mc+mp)+mc*mp*L^2;
A=[0 1 0 0;
    0 -(I+mp*L^2)*bc/D mp^2*L^2*g/D -mp*L*bp/D;
    0 0 0 1;
    0 -mp*L*bc/D (mc+mp)*mp*L*g/D -(mc+mp)*bp/D];

B=[0 (I+mp*L^2)/D 0 mp*L/D]';
C=[1 0 0 0;
    0 0 1 0];
D=0;

F=[0 0;
   1 0;
   0 0;
   0 1];

E=[0;0];

%LQR with more weights on the state settling
Q=C'*C;
R=1;
Q(1,1) = 5000;%really fast cart settling 
Q(3,3) = 100;
k=lqr(A,B,Q,R)

%2nd degree of freedom
%calculating Nbar to make the cart stop at reference position
s = size(A,1);
Z = [zeros([1,s]) 1];
N = inv([A,B;C(1,:),D])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar=Nu + k*Nx

%feedback of state space system
sysFB=ss(A-B*k,B*Nbar,C,D);

%creating input and output
t = 0:0.01:5;
r =0.1*ones(size(t));

r(1:50)=0.25;
%r(51:300)=t(51:300)*0.25-0.25;
%r(300:501)=-0.25;
[y,t,x]=lsim(sysFB,r,t,[0.25 0 0 0]);%max initial condition for x3=0.6925

u=-k*x'+Nbar*r;

%plotting
figure(1)
set(gcf,'Position',[10 50 800 400])
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')
figure(2)
set(gcf,'Position',[850 50 700 400])
plot(t,u)
title('control input with LQR Control')
input=[t, u'];
if AnimationEnable==1
    %simulation of the system
    figure(3)
    set(gcf,'Position',[450 700 700 400])
    drawpend(x(1,:))
    pause(0.5)
    for i=1:length(t)
        clf;
        axis equal
        drawpend(x(i,:))
        pause(t(2)-t(1));
    end
    hold off
end