% Log some cool stuff
disp("MatLAB Accelerometer Data Analyzer");
disp("Version 1.0.2");

% Clear everything
clear;
clc;
clf;

% Plot configuration
plotTitle       = 'ADC plot';
xLabel          = 'Elapsed Time (s)';
yLabel          = 'g [m/s^2]';
legend1         = 'Accelerometer Value'; % 
yMax            = 2.5;
yMin            = -2.5;
plotGrid        = 'on';

refreshDelay    = 0.0001;
speedValue      = 30;

%Define function values
time        = 0;
data        = 0;
count       = 0;

% Draw the plot
plotGraph = plot(time, data, '-r');
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

% Maximize figure window
drawnow;
set(get(handle(gcf), 'JavaFrame'), 'Maximized', 1);

pause(0.5);

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

% Enable Motor
fwrite(bmodule, uint8(speedValue));

% Enable timer
tic

while ishandle(plotGraph) %Loop when Plot is Active will run until plot is closed
    
    b = fread(bmodule, 1);
    
    count = count + 1;

    time(count) = toc;
    data(count) = b(1) / 256 * 5 - 2.5;
    
    %This is the magic code 
    %Using plot will slow down the sampling time.. At times to over 20
    %seconds per sample!
    
    set(plotGraph, 'XData', time, 'YData', data);
    axis([toc-2 time(count) yMin yMax]);

    % Update the graph
    drawnow;
    pause(refreshDelay);
    
    % Flush the buffer
    
    %flushinput(bmodule);
    
    if bmodule.BytesAvailable >= 512
        disp("Buffer overflow.");
    end

end

fclose(bmodule);