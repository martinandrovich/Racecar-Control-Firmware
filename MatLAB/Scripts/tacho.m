% Global definitions
run('definitions.m');

% Tachometer Logging
disp("Tachometer Logging [Ticks/s]");
disp("Version 1.1.0");

% Plot configuration
plotTitle       = 'Tachometer Plot';
xLabel          = 'Elapsed Time [s]';
yLabel          = 'Ticks [-]';
legend1         = 'Tachometer Value';
yMax            =  inf;
yMin            =  0;
plotGrid        = 'on';

% Connect to Bluetooth Module
fopen(bmodule);
fprintf('Connection established; starting data logging.');

% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.Tachometer);

% Start vehicle
UnitController.setDutyCycle(dutyCycle);

% Enable timer
tic

% Log data
while tachoVal < logDistance
    
   dataBytes = fread(bmodule, 2);   
   
   dataLong = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
   tachoVal = dataLong;
   disp(tachoVal);
   data(count) = dataLong;
   
   count = count + 1;
  
end

% Stop vehicle & broadcasting
UnitController.setDutyCycle(0);
UnitController.setBroadcastMode(broadcastModes.Disabled);

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