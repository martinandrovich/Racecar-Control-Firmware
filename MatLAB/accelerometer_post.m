% Log some cool stuff
disp("MatLAB Accelerometer POST Data Analyzer");
disp("Version 1.0.3");

% Clear everything
clear;
clc;
clf;
close(gcf);

% Plot configuration
plotTitle       = 'ADC plot';
xLabel          = 'Elapsed Time (s)';
yLabel          = 'g [m/s^2]';
legend1         = 'Accelerometer Value (Raw)';
legend2         = 'Filter: Moving Average';
legend3         = 'Filter: Moving Mean';
yMax            =  2;
yMin            = -2;
plotGrid        = 'on';

% Definitions
broadcastModes  = struct(...
                    'Disabled',         0,      ...                    
                    'All',              8,      ...
                    'Tachometer',       24,     ...
                    'Finishline',       56,     ...
                    'Accelerometer',    40      ...
                  );           

logDuration     = 10;
movingAvgSize   = 32;
timerFreq       = 1;
movingAvg       = (1/movingAvgSize) * ones(1, movingAvgSize);

time            = 0;
data            = 0;
count           = 0;

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

disp('Connection established; starting data logging.');

% Set broadcasting mode
setBroadcastMode(broadcastModes.Accelerometer);

% Start vehicle
setDutyCycle(101);
%fwrite(bmodule, uint8(speedValue));

% Enable timer
tic

% Log data
while toc < logDuration
    
   dataBytes = fread(bmodule, timerFreq);   
   data(1+count*timerFreq:timerFreq*(count+1)) = dataBytes(1:timerFreq);
   count = count + 1;
   
   if toc > (logDuration - 0.5)
       setDutyCycle(0);
       %fwrite(bmodule, uint8(1));
   end
   
end

% Calculate elapsed time
timeWaited = toc;
timeActual = timeWaited/length(data);
timeElapsed = 0 + timeActual : timeActual : timeWaited;

% Calculate data & filters
data = (data / 256) * 4 - 2;
dataAvg = filter(movingAvg, 1, data);
dataMean = movmean(data, movingAvgSize);

% Plot data
plotGraph = plot(timeElapsed, data, '-', timeElapsed, dataAvg, '-', timeElapsed, dataMean, '-g');
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

% Close connection
fclose(bmodule);

% -----------------------------------------------------------------------------------------------------------------------------------

% Device Control Functions

function setDutyCycle(value)
    bmodule = evalin('base', 'bmodule');
    fwrite(bmodule, uint8(85));
    fwrite(bmodule, uint8(16));
    fwrite(bmodule, uint8(value));
end

function setBroadcastMode(mode)
    bmodule = evalin('base', 'bmodule');
    fwrite(bmodule, uint8(85));
    fwrite(bmodule, uint8(20));
    fwrite(bmodule, uint8(mode));
end