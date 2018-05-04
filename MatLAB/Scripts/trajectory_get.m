% Connect to Bluetooth Module
fopen(bmodule);
fprintf('\nConnection established; getting trajectory.\n');

% Get trajectory
UnitController.getTrajectory();
pause(1);
count = 1;

while (true)
    
    if (bmodule.BytesAvailable >= 2)
        
        dataBytes = fread(bmodule, 2);
        dataLong = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
        
        % Exit if EoT
        if (dataLong == 65535)
            break;
        end
        
        % Extract turn bit
        dataTrajectory(count, 2) = bitget(dataLong, 16);
        
        % Extract Tachometer value
        dataTrajectory(count, 1) = bitset(dataLong, 16, 0);
        
        count = count + 1;
        
    end
    
end

fprintf('\nTrajectory (%d BYTES) saved in array.\n', count);

% Close connection
fclose(bmodule);