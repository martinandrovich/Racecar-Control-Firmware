% Global definitions
run('definitions.m');

% Tachometer & Accelerometer Logging
disp("Velocity [Ticks/S]");
disp("Version 1.0.0");

% Connect to Bluetooth Module
fopen(bmodule);
fprintf('\nConnection established; starting data logging.');


% Set broadcasting mode
UnitController.setBroadcastMode(broadcastModes.Velocity);

% Start vehicle
UnitController.setDutyCycle(dutyCycle);

while (true)
    
    if (bmodule.BytesAvailable >= 2)
        
        dataBytes = fread(bmodule, 2);
        dataLong = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
        
        dataVelocity = round(dataLong * 1.92, 3);
        
        %fprintf(repmat('\b', 1, lastsize));
        %lastsize = fprintf('%Velocity: %d cm/s', dataVelocity);
        
        fprintf('Velocity: %d cm/s\n', dataLong);
        
    end
    
end

% Close connection
fclose(bmodule);