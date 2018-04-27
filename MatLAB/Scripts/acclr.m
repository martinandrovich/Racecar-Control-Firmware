% Global definitions
run('definitions.m');

% Accelerometer Logging
disp("Filtered Accelerometer Logging [B/count]");
disp("Version 1.1.2");

% Plot configuration
plotTitle       = 'Accelerometer Plot';
xLabel          = 'Elapsed Counts';
yLabel          = 'Byte [B]';
legend1         = 'Accelerometer Byte Value (ASM Filtered)';
yMax            =  255;
yMin            =  0;
plotGrid        = 'on';

% Connect to Bluetooth Module
fopen(bmodule);
fprintf('\nConnection established; starting data logging.\n');

% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.Accelerometer);

% Start vehicle
UnitController.setDutyCycle(dutyCycle);

% Enable timer
tic

% Log data
while toc < logDuration
   
   if (bmodule.BytesAvailable) 
       dataBytes = fread(bmodule, 1);   
       data(count) = dataBytes;
       count = count + 1;
   end
   
   if (toc > (logDuration - bufferDelay)) && stateEnabled
        UnitController.setDutyCycle(0);
        UnitController.setBroadcastMode(broadcastModes.Disabled);
   end
   
end

% Trim data
data(count+1:prealloc) = [];

% Calculate elapsed time
timeWaited = toc;
timeActual = timeWaited/length(data);
timeElapsed = 0 + timeActual : timeActual : timeWaited;

% Plot data
plotGraph = plot(data, '-');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1);
axis([0 length(data) yMin yMax]);
grid(plotGrid);

% Maximize figure window
drawnow;
set(get(handle(gcf), 'JavaFrame'), 'Maximized', 1);

% Close connection
fclose(bmodule);