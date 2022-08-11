function drawpend(state)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
x=state(1);
th=state(3);
y=0;

cartW=0.3;
cartH=0.15;
L=0.6;

pendx = x - L*sin(th);
pendy = y + L*cos(th);




axis([-1 1 -1 1]);hold on
plot([-10 10],[0 0],'k','LineWidth',2)
rectangle('Position',[x-cartW/2,y-cartH/2,cartW,cartH],'FaceColor',[0 0.4470 0.7410],'Curvature',.1)
plot([x pendx],[y pendy],'color',[0.5 0 0.2],'LineWidth',5); % Draw pendulum

drawnow, hold off
end

