% Global definitions
run('definitions.m');

% Tachometer & Accelerometer Logging
disp("Threshold Mapping [Acclr Turn/Ticks]");
disp("Version 1.1.1");

% Plot configuration
plotTitle       = 'Accelerometer Turn/Tachometer Plot';
xLabel          = 'Ticks [ti]';
yLabel          = 'Turn Value';
legend1         = 'Accelerometer Turn Value (ASM Filtered)';
yMax            =  2;
yMin            =  0;
plotGrid        = 'on';

% Connect to Bluetooth Module
fopen(bmodule);
fprintf('\nConnection established; starting data logging.\n');
    
% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.All);

% Start vehicle
UnitController.setDutyCycle(dutyCycle);

% Enable timer
tic

% Log data
while tachoVal < logDistance*logLaps

   if (bmodule.BytesAvailable >= 3)     
       dataBytes = fread(bmodule, 3);
       tachoVal  = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
       
       dataAcclr(count) = dataBytes(3);
       dataTacho(count) = tachoVal;

       count = count + 1;
   end
   
end

% Stop vehicle & broadcasting
UnitController.setDutyCycle(0);
UnitController.setBroadcastMode(broadcastModes.Disabled);

%Empty buffer
pause(bufferDelay);

while bmodule.BytesAvailable

   if (bmodule.BytesAvailable >= 3)     
       dataBytes = fread(bmodule, 3);
       tachoVal  = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
       
       dataAcclr(count) = dataBytes(3);
       dataTacho(count) = tachoVal;

       count = count + 1;
   end
   
end

% Plot data
plotGraph = plot(dataTacho, dataAcclr, '-');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
legend(legend1);
axis([0 dataTacho(end) yMin yMax]);
grid(plotGrid);

% test = [73, 126, 217, ];
% t = unique(tachometer(accelerometer==125));

% Maximize figure window
drawnow;
set(get(handle(gcf), 'JavaFrame'), 'Maximized', 1);

% Close connection
fclose(bmodule);