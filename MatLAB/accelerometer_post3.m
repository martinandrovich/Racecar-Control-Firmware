% Log some cool stuff
disp("MatLAB Accelerometer POST Data Analyzer");
disp("Version 1.0.4");

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
legend2         = 'Filter: Moving Mean';
yMax            = 2;
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
dataLength      = 1;

time            = 0;
data            = 0;
count           = 0;

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

% Set broadcasting mode
prompt = 'Please enter a mode (1-4): ';
broadcastMode = input(prompt);

switch broadcastMode
    
    % ALL
    case 1
        disp('Yet not implemented.');
        return;
    
    % TACHOMETER
    case 2
        setBroadcastMode(broadcastModes.Tachometer);
        dataLength = 2;
        dataLong = uint16(0);
        
    % FINISHLINE
    case 3
        disp('Yet not implemented.');
        return;
        
    % ACCELEROMETER
    case 4
        setBroadcastMode(broadcastModes.Accelerometer);
        dataLength = 1;
    
    otherwise
        disp('Mode does not exist.');
        fclose(bmodule);
        return;
        
end

disp('Connection established; starting data logging.');

% Start vehicle
setDutyCycle(101);

% Enable timer
tic

% Log data
while toc < logDuration
    
   dataBytes = fread(bmodule, dataLength);
   
   switch broadcastMode
       
    % ACCELEROMETER
    case 4
        data(1+count*timerFreq:timerFreq*(count+1)) = dataBytes(1:timerFreq);
        
    % TACHOMETER
    case 2
        dataLong = (bitshift(dataBytes(1), 8, 'uint8')) | uint8(dataBytes(2));
        data(count+1) = dataLong;
        
   end
   
   count = count + 1;
   
   % Stop vehicle
   if toc > (logDuration - 0.5)
       setDutyCycle(0);
   end
   
end

% Calculate elapsed time
timeWaited = toc;
timeActual = timeWaited/length(data);
timeElapsed = 0 + timeActual : timeActual : timeWaited;

% Calculate data & filters

switch broadcastMode
    
    case 2
        plotGraph = plot(timeElapsed, data, '-');
        
    case 4
        data = (data / 256) * 4 - 2;
        dataMean = movmean(data, movingAvgSize);
        plotGraph = plot(timeElapsed, data, '-', timeElapsed, dataMean, '-');
        
end
    
% Plot data
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