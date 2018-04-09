% Log some cool stuff
disp("MatLAB Accelerometer POST Data Analyzer");
disp("Version 1.0.0");

% Clear everything
clear;
clc;

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

% Definitions
movingAvgSize   = 64;
timerFreq       = 256;
movingAvg       = (1/movingAvgSize) * ones(1, movingAvgSize);

refreshDelay    = 0.0001;
speedValue      = 40;

time            = 0;
data            = 0;
count           = 0;

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

% Start vehicle
fwrite(bmodule, uint8(speedValue));

disp('Connection established; starting data logging.');

while count < 7
    
   dataBytes = fread(bmodule, timerFreq);   
   data(1+count*timerFreq:timerFreq*(count+1)) = dataBytes(1:timerFreq);
   
   count = count + 1;

end

% Calculate data & filters
data = (data / 256) * 5 - 2.5;
dataAvg = filter(movingAvg, 1, data);
dataMean = movmean(data, movingAvgSize);

% Calculate spent time
timeWaited = toc;
timeActual = timeWaited/length(data);
timeElapsed = 0 + timeActual : timeActual : timeWaited;

% Plot data
clf;
plotGraph = plot(timeElapsed, data, '--', timeElapsed, dataAvg, '-', timeElapsed, dataMean, '-g');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1, legend2, legend3);
axis([0 timeWaited yMin yMax]);
grid(plotGrid);

% Maximize figure window
drawnow;
set(get(handle(gcf), 'JavaFrame'), 'Maximized', 1);

% Stop vehicle & disconnect
fwrite(bmodule, uint8(1));
fclose(bmodule);