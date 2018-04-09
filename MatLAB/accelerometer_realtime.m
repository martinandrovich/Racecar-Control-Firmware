% Log some cool stuff
disp("MatLAB Accelerometer REALTIME Data Analyzer");
disp("Version 1.0.3");

% Clear everything
clear;
clc;
clf;

% Definitions
refreshDelay    = 0.0001;
speedValue      = 40;
movingAvgSize   = 16;

time            = 0;
data            = 0;
count           = 0;

% Plot configuration
plotTitle       = 'ADC plot';
xLabel          = 'Elapsed Time (s)';
yLabel          = 'g [m/s^2]';
legend1         = 'Accelerometer Value (Raw)';
legend2         = 'Filter: Moving Average';
legend3         = 'Filter: Moving Mean';
yMax            = 2.5;
yMin            = -2.5;
plotGrid        = 'on';

% Draw the plot
plotGraph = plot(time, data, '-r', time, data, '-', time, data, '-g');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1, legend2, legend3);
axis([yMin yMax yMin yMax]);
grid(plotGrid);

% Setup filters
movingAvg = (1/movingAvgSize) * ones(1, movingAvgSize);

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

% Loop while plot is active
while ishandle(plotGraph)
    
    dataBytes = fread(bmodule, 1);
    
    count = count + 1;

    time(count) = toc;
    data(count) = (dataBytes(1) / 256) * 5 - 2.5;
    
    dataMean = movmean(data, movingAvgSize);
    dataAvg = filter(movingAvg, 1, data);
    
    set(plotGraph, {'xdata'}, {time;time;time}, {'ydata'}, {data;dataAvg;dataMean});
    axis([toc-4 time(count) yMin yMax]);

    % Update the graph
    %drawnow;
    pause(refreshDelay);
    
    % Flush the buffer
    %flushinput(bmodule);
    
    disp(bmodule.BytesAvailable);
    
    if bmodule.BytesAvailable >= 512
        disp("Buffer overflow.");
    end

end

fclose(bmodule);