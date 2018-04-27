% MatLAB Accelerometer POST Data Analyzer
disp("MatLAB Threshold POST Data Analyzer");
disp("Version 1.0.2");

% Clear everything
clear;
clc;
clf;
close(gcf);

% Plot configuration
plotTitle       = 'Accelerometer/Tachometer Plot';
xLabel          = 'Ticks [ti]';
yLabel          = 'Byte Value [B]';
legend1         = 'Accelerometer Value (ASM Filtered)';
yMax            = 2;
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
count           = 1;
stateEnabled    = false;

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

disp('Connection established; starting data logging.');

% Set broadcasting mode
setBroadcastMode(broadcastModes.All);

% Start vehicle
setDutyCycle(90);

% Enable timer
tic

% Log data
while toc < logDuration

   if (bmodule.BytesAvailable >= 4)
       dataBytes = fread(bmodule, 4);
       dataLong = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
       accelerometer(count) = dataBytes(3);
       tachometer(count) = dataLong;
       count = count + 1;
   end
   
   if (toc > (logDuration - 0.5)) && stateEnabled
        setDutyCycle(0);
        setBroadcastMode(broadcastModes.Disabled);
   end
end

i = 1;
j = 1;
k = 2;

newPlotAcc = 0;
newPlotTach = 0;

while i < (length(accelerometer) - 1)
    
    if accelerometer(i) ~= accelerometer(i+1)
        
        disp(i);
        disp(accelerometer(i));
        disp(accelerometer(i+1));
        disp(accelerometer(i) ~= accelerometer(i+1));
        
        newPlotAcc(k) = accelerometer(i);
        newPlotTach(k) = tachometer(i) - 8 * j;
        
        k = k + 1;
        
        newPlotAcc(k) = accelerometer(i+1) ;
        newPlotTach(k) = tachometer(i+1) - 8 * j;
        
        k = k + 1;
        j = j + 1;
        
        if (j == 3)
            j = 1;
        end   
        
    end
    
    i = i + 1;
    
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
axis([0 tachometer(end) yMin yMax]);
grid(plotGrid);

plotGraph = plot(newPlotTach, newPlotAcc, '-');

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
    
    if (mode == 0)
        assignin('base', 'stateEnabled', false);
    else
        assignin('base', 'stateEnabled', true);
    end
    
    fwrite(bmodule, uint8(85));
    fwrite(bmodule, uint8(20));
    fwrite(bmodule, uint8(mode));
end