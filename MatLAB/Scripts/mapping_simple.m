% Global definitions
run('definitions.m');

% Tachometer & Accelerometer Logging
disp("Simple Mapping [Acclr/Ticks]");
disp("Version 1.1.1");

% Plot configuration
plotTitle       = 'Accelerometer/Tachometer Plot';
xLabel          = 'Ticks [ti]';
yLabel          = 'Byte Value [B]';
legend1         = 'Accelerometer Value (ASM Filtered)';
yMax            =  255;
yMin            =  0;
plotGrid        = 'on';

% Setup Bluetooth Module
bmodule = Bluetooth('RNBT-E2A9', 1);
fopen(bmodule);

disp('Connection established; starting data logging.');
    
% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.All);

% Start vehicle
UnitController.setDutyCycle(dutyCycle);

% Enable timer
tic

% Log data
while tachoVal < logDistance

   if (bmodule.BytesAvailable >= 4)     
       dataBytes = fread(bmodule, 4);
       tachoVal  = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
       
       dataAcclr(count) = dataBytes(3);
       dataTacho(count) = tachoVal;

       count = count + 1;
   end
   
end

% Stop vehicle & broadcasting
UnitController.setDutyCycle(0);
UnitController.setBroadcastMode(broadcastModes.Disabled);
pause(bufferDelay);

% Empty buffer
while (bmodule.BytesAvailable)
   if (bmodule.BytesAvailable >= 4)
       
       dataBytes = fread(bmodule, 4);
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