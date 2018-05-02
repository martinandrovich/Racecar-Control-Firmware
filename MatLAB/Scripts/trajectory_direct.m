% Reset values
offset          = 0;
tachoVal        = 0;
boolTurn        = false;

% Connect to Bluetooth Module
fopen(bmodule);
fprintf('\nConnection established; lets go!\n');

% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.Tachometer);
pause(bufferDelay);

% Start vehicle
UnitController.setDutyCycle(speedAccelerate);

while tachoVal < dataMapping(end, 1)
    
    if (bmodule.BytesAvailable >= 2)
        dataBytes = fread(bmodule, 2);
        tachoVal  = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
    else
        continue;
    end
    
    if(find(dataMapping(:,1) == tachoVal))
        
        dataValue = dataMapping(find(dataMapping(:,1) == tachoVal), 2);
       
       if dataValue == 1 && ~boolTurn
           fprintf('Break at: %d\n\n', tachoVal);
           boolTurn = true;
           UnitController.setDutyCycle(0);
           pause(1);
           UnitController.setDutyCycle(speedBrake);
       end
       
       if dataValue == 0 && boolTurn
           fprintf('Accelerate at: %d\n\n', tachoVal);
           boolTurn = false;
           UnitController.setDutyCycle(speedAccelerate);
       end
        
    end
    
end

% Stop vehicle & broadcasting
UnitController.setDutyCycle(0);
UnitController.setBroadcastMode(broadcastModes.Disabled);

% Close connection
fclose(bmodule);
