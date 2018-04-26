% Global definitions
run('definitions.m');

% Raw Accelerometer Logging
disp("Raw Accelerometer Logging [B/s]");
disp("Version 1.1.0")

% Plot configuration
plotTitle       = 'Accelerometer Plot';
xLabel          = 'Elapsed Time [s]';
yLabel          = 'Byte [B]';
legend1         = 'Accelerometer Byte Value (Raw)';
legend2         = 'Filter: Moving Mean';
yMax            =  255;
yMin            =  0;
plotGrid        = 'on';

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

disp('Connection established; starting data logging.');

% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.Accelerometer);

% Start vehicle
UnitController.setDutyCycle(dutyCycle);

% Enable timer
tic

% Log data
while toc < logDuration
    
   dataBytes = fread(bmodule, 1);   
   data(1+count*1:1*(count)) = dataBytes(1:1);
   count = count + 1;
   
   if toc > (logDuration - 0.5)
       setDutyCycle(0);
   end
   
end

% Stop broadcasting
disp('Stopping data logging...');
UnitController.setBroadcastMode(broadcastModes.Disabled);

pause(0.2);

% Read last bytes from buffer
while (bmodule.BytesAvailable)
   dataBytes = fread(bmodule, 1);   
   data(1+count*1:1*(count)) = dataBytes(1:1);
   count = count + 1;   
end

% Calculate elapsed time
timeWaited = toc;
timeActual = timeWaited/length(data);
timeElapsed = 0 + timeActual : timeActual : timeWaited;

% Calculate filter
%data = (data / 256) * 4 - 2;
dataMean = movmean(data, movingAvgSize);

% Plot data
plotGraph = plot(timeElapsed, data, '-', timeElapsed, dataMean, '-g');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1, legend2);
axis([0 timeWaited yMin yMax]);
grid(plotGrid);

% Maximize figure window
drawnow;
set(get(handle(gcf), 'JavaFrame'), 'Maximized', 1);

% Close connection
fclose(bmodule);