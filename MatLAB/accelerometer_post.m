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

% Setup variables
movingavgsize   = 64;
freq            = 256;
avgsize = 1/movingavgsize * ones(1, movingavgsize);

refreshDelay    = 0.0001;
speedValue      = 40;

%Define function values
time            = 0;
data            = 0;
count           = 0;

pause(0.5);

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

% Enable Motor
fwrite(bmodule, uint8(speedValue));

disp('Lets go!');

while count < 7
    
   dataBytes = fread(bmodule, freq);   
   data(1+count*freq:freq*(count+1)) = dataBytes(1:freq);
   
   count = count + 1;

end

time_waited = toc;
time_actual = time_waited/length(data);
timewaited = 0 + time_actual : time_actual : time_waited;

data = (data / 256) * 5 - 2.5;

dataAvg = filter(avgsize, 1, data);
dataMean = movmean(data, movingavgsize);

t = timewaited;

clf;

plotGraph = plot(t, data, '--', t, dataAvg, '-', t, dataMean, '-g');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1, legend2, legend3);
axis([0 time_waited yMin yMax]);
grid(plotGrid);

% Maximize figure window
drawnow;
set(get(handle(gcf), 'JavaFrame'), 'Maximized', 1);

fwrite(bmodule, uint8(1));

fclose(bmodule);