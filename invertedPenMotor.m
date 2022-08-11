clc;clear;
close all;
AnimationEnable=0;
SaveAnimation=1;
%system parameters
mc=0.75;
mp=0.5;
l=0.6;L=0.3;
I=0.01;
bc=0.1;
bp=0.005;
g=9.81;

Jm=8.0000e-06;
bm=0.0003;
Rm=0.5;
Lm=0.7680;
Km=0.1026;
n=40; %it was a design parameter
r=0.04;

D=I*(mc+mp)+mc*mp*L^2;
A=[ 0 1 0 0 0;
    0 -bc*(I+mp*L^2)/D mp^2*L^2*g/D -mp*L*bp/D n*Km*(I+mp*L^2)/(D*r);
    0 0 0 1 0;
    0 -bc*mp*L/D mp*L*g*(mp+mc)/D -bp*(mp+mc)/D n*mp*L*Km/(r*D);
    0 -Km/Lm 0 0 -Rm/Lm];

B=[0 0 0 0 1/Lm]';
C=[1 0 0 0 0;
    0 0 1 0 0
    0 0 0 0 1];
D=0;
%sys=ss(A,B,C,D);

Q=C'*C;
Q(1,1)=2700;
R=4000;
k=lqr(A,B,Q,R)

s = size(A,1);
Z = [zeros([1,s]) 1];
N = inv([A,B;C(1,:),D])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar=Nu + k*Nx

sysFB=ss(A-B*k,B*Nbar,C,D);
%[Tsx1 Tsx2]=stepinfo(sysFB).SettlingTime

%creating input and output
t = 0:0.01:10;
%reference input for Cart position
r =0*ones(size(t));
r(350:650)=0.4;
r(650:1001)=0.0;


[y,t,x]=lsim(sysFB,r,t,[0 0 -0.1 -0.4 0]);%max initial condition for x3=0.24


%Umax=max(abs(u))
%Xmax=max(abs(x))

u=-k*x'+Nbar*r; 

%plotting
figure(1)
set(gcf,'Position',[10 200 800 400])
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')
figure(2)
set(gcf,'Position',[850 50 700 400])
plot(t,u)
title('control input with LQR Control')


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
if SaveAnimation==1
    myVideo = VideoWriter('Simulation'); %open video file
    myVideo.FrameRate = 100;  %can adjust this, 5 - 10 works well for me
    open(myVideo)


    %simulation of the system
    figure(1)
    set(gcf,'Position',[450 700 700 400])
    drawpend(x(1,:))
    pause(0.5)
    for i=1:size(t)
        clf;
        axis equal
        drawpend(x(i,:))
        pause(0.01);
        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);
    end
    hold off
    close(myVideo)
end
