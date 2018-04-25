% MatLAB Accelerometer POST Data Analyzer
disp("MatLAB Accelerometer POST Data Analyzer");
disp("Version 1.0.5");

% Clear everything
clear;
clc;
clf;
close(gcf);

% Plot configuration
plotTitle       = 'Accelerometer Plot';
xLabel          = 'Elapsed Time [s]';
yLabel          = 'Byte [B]';
legend1         = 'Accelerometer Byte Value (ASM Filtered)';
yMax            =  255;
yMin            =  0;
plotGrid        = 'on';

% Definitions
broadcastModes  = struct(...
                    'Disabled',         0,      ...                    
                    'All',              8,      ...
                    'Tachometer',       24,     ...
                    'Finishline',       56,     ...
                    'Accelerometer',    40      ...
                  );           

logDuration     = 20;
timerFreq       = 1;

data            = 0;
count           = 0;

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

disp('Connection established; starting data logging.');

% Set broadcasting mode
setBroadcastMode(broadcastModes.Accelerometer);

% Start vehicle
setDutyCycle(90);

% Enable timer
tic

% Log data
while toc < logDuration
    
   dataBytes = fread(bmodule, 1);   
   data(1+count*timerFreq:timerFreq*(count+1)) = dataBytes(1:timerFreq);
   count = count + 1;
   
   if toc > (logDuration - 0.5)
       setDutyCycle(0);
   end
   
end

disp('Stopping data logging.');
setBroadcastMode(broadcastModes.Disabled);

pause(0.2);

while (bmodule.BytesAvailable)
   dataBytes = fread(bmodule, 1);   
   data(1+count*timerFreq:timerFreq*(count+1)) = dataBytes(1:timerFreq);
   count = count + 1;   
end

% Calculate elapsed time
timeWaited = toc;
timeActual = timeWaited/length(data);
timeElapsed = 0 + timeActual : timeActual : timeWaited;

% Plot data
plotGraph = plot(timeElapsed, data, '-');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1);
axis([0 logDuration yMin yMax]);
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