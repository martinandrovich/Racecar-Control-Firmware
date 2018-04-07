% Clear everything
clear;
clc;
clf;

% Plot Configuration
plotTitle = 'ADC plot';
xLabel = 'Elapsed Time (s)';
yLabel = 'g [m/s^2]';
legend1 = 'Accelerometer Value'; % 
yMax = 2.5;
yMin = -2.5;
plotGrid = 'on';
refreshDelay = 0.01;

%Define function values
time = 0;
data = 0;
count = 0;

% Draw the Plot
plotGraph = plot(time, data, '-r');
linkdata on;
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1);
axis([yMin yMax yMin yMax]);
grid(plotGrid);
set(gcf, 'Position', get(0, 'Screensize'));

pause(1);

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9',1);
fopen(bmodule);

% Enable Motor
fwrite(bmodule, uint8(30));

% Enable timer
tic

while ishandle(plotGraph) %Loop when Plot is Active will run until plot is closed
    
    byteFirst = fread(bmodule, 1);
    
    if firstByte ~= uint8(20)
        continue;
    end
    
    while(bmodule.BytesAvailable ~= 2)
        % Do Nothing
    end
        
    bytePair = fread(bmodule, 2);
    %b = fread(bmodule, 2);
    
    count = count + 1;

    b = fread(bmodule, 2);

    time(count) = toc;
    data(count) = (5/1024)*(b(2)*256+b(1))-2.5;
    %data = b(1)*256;
    %This is the magic code 
    %Using plot will slow down the sampling time.. At times to over 20
    %seconds per sample!
    set(plotGraph, 'XData', time, 'YData', data);
    axis([toc-2 time(count) yMin yMax]);

    % Update the graph
    %drawnow;
    %pause(refreshDelay);
    
    % Flush the buffer
    %flushinput(bmodule);

end

fclose(bmodule);