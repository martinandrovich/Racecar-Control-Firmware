% Tachometer & Accelerometer Logging
disp("Simple Trajectory Test");
disp("Version 1.0.0");

% Reset variables
tachoVal        = 0;
boolTurn        = false;

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
    
    if (tachoVal + offsetVal) == length(dataTacho)
        UnitController.setDutyCycle(0);
        break;
    end
    
    if dataAcclr(tachoVal + offsetVal) == 1 && ~boolTurn
        boolTurn = true;
        UnitController.setDutyCycle(speedBrake);
        %fprintf('\nBreak\nTachometer = %d\n', tachoVal);
    end
    
    if dataAcclr(tachoVal + offsetVal) == 0 && boolTurn
        boolTurn = false;
        UnitController.setDutyCycle(speedAccelerate);
        %fprintf('\nAccelerate\nTachometer = %d\n', tachoVal);
    end
    
end

% Stop vehicle & broadcasting
UnitController.setDutyCycle(0);
UnitController.setBroadcastMode(broadcastModes.Disabled);
pause(bufferDelay);

% Close connection
fclose(bmodule);