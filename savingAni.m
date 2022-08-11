myVideo = VideoWriter('Final'); %open video file
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