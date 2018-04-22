% MatLAB Accelerometer POST Data Analyzer
disp("MatLAB Accelerometer POST Data Analyzer");
disp("Version 1.0.4");

% Clear everything
clear;
clc;
clf;
close(gcf);

% Plot configuration
plotTitle       = 'Accelerometer Plot';
xLabel          = 'Elapsed Time [s]';
yLabel          = 'g [m/s^2]';
legend1         = 'Accelerometer Value (ASM Filtered)';
yMax            = 255;
yMin            = 0;
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
timerFreq       = 1;

accelerometer   = uint8(0);
tachometer      = uint16(0);
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

   if (bmodule.BytesAvailable >= 4)
   dataBytes = fread(bmodule, 4);
   accelerometer(count+1) = dataBytes(3);
   tachometer(count+1) = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
   count = count + 1;
   end
   
   if toc > (logDuration - 0.5)
       setDutyCycle(0);
       setBroadcastMode(broadcastModes.Disabled);
   end
   
   
end

% Plot data
plotGraph = plot(tachometer, accelerometer, '-');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1);
axis([0 length(tachometer) yMin yMax]);
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