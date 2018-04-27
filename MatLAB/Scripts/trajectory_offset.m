% Tachometer & Accelerometer Logging
disp("Simple Trajectory Test");
disp("Version 1.0.0");

% Reset variables
tachoVal        = 0;
boolTurn        = false;

% Apply offset filter
[dataTachoOffset, dataAcclrOffset] = mappingOffSet(dataTacho, dataAcclr);

% Connect to Bluetooth Module
fopen(bmodule);
fprintf('\nConnection established; lets go!\n');

% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.Tachometer);
pause(bufferDelay);

UnitController.setDutyCycle(speedAccelerate);

while tachoVal < logDistance
    
   if (bmodule.BytesAvailable)
        dataBytes = fread(bmodule, 2);
        tachoVal = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
   end
    
   if(find(dataTachoOffset == tachoVal))
       
       dataValue = dataAcclrOffset(find(dataTachoOffset == tachoVal));
       
       if dataValue == 1 && ~boolTurn
           boolTurn = true;
           UnitController.setDutyCycle(speedBrake);
       end
       
       if dataValue == 0 && boolTurn
           boolTurn = false;
           UnitController.setDutyCycle(speedAccelerate);
       end
       
   end
    
end

% Stop vehicle & broadcasting
UnitController.setDutyCycle(0);
UnitController.setBroadcastMode(broadcastModes.Disabled);
pause(bufferDelay);

% Close connection
fclose(bmodule);