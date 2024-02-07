function makeVideo(F,filename)
    testtime = datestr(now,'mm-dd-yy-HHMMSS');
    video_filename = sprintf('%s-%s', filename,testtime);
    % % % create the video writer with 30 fps
    if ispc
        vidForm = 'MPEG-4';
    else
        vidForm = 'Motion JPEG AVI';
    end
      writerObj = VideoWriter(video_filename,vidForm);
      writerObj.FrameRate = 30;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i= 2:length(F)
        % convert the image to a frame
        frame = F(i);
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
end