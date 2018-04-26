% MatLAB Tachometer POST Data Analyzer
disp("MatLAB Tachometer POST Data Analyzer");
disp("Version 1.0.3");

% Clear everything
clear;
clc;
clf;
close(gcf);

% Plot configuration
plotTitle       = 'Tachometer Plot';
xLabel          = 'Elapsed Time [s]';
yLabel          = 'Ticks';
legend1         = 'Tachometer Value';
yMax            =  inf;
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

logDuration     = 10;
logDistance     = 290;
data            = 0;
dataLong        = uint16(0);
count           = 1;

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

disp('Connection established; starting data logging.');

% Set broadcasting mode
setBroadcastMode(broadcastModes.Tachometer);

% Start vehicle
setDutyCycle(120);

% Enable timer
tic

% Log data
while dataLong < logDistance
    
   dataBytes = fread(bmodule, 2);   
   
   dataLong = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
   %disp(dataLong);
   data(count) = dataLong;
   
   count = count + 1;
  
end

% Stop vehicle & broadcasting
setDutyCycle(0);
setBroadcastMode(broadcastModes.Disabled);

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