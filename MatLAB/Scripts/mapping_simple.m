% Global definitions
run('definitions.m');

% Tachometer & Accelerometer Logging
disp("Simple Mapping [Acclr/Ticks]");
disp("Version 1.1.0");

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
       
       if count == 1
           
           dataAcclr(count) = dataBytes(3);
           dataTacho(count) = tachoVal;

           count = count + 1;
           
           continue;
       
       elseif dataTacho(count) ~= tachoVal
       
           dataAcclr(count) = dataBytes(3);
           dataTacho(count) = tachoVal;

           count = count + 1;
       
       end
   end
   
end

% Stop vehicle & broadcasting
UnitController.setDutyCycle(0);
UnitController.setBroadcastMode(broadcastModes.Disabled);

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