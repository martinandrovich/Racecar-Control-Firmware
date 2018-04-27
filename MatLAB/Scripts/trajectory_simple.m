% Tachometer & Accelerometer Logging
disp("Simple Trajectory Test");
disp("Version 1.0.0");

boolTurn        = false;
offsetVal       = 10;
speedBrake      = 60;
speedAccelerate = 90;


for i = 1:length(dataTacho)
    
    if (i + offsetVal) == length(dataTacho)
        break;
    end
    
    if dataAcclr(i + offsetVal) == 1 && ~boolTurn
        boolTurn = true;
        fprintf('\nBreak\nTachometer = %d\n', i);
    end
    
    if dataAcclr(i + offsetVal) == 0 && boolTurn
        boolTurn = false;
        fprintf('\nAccelerate\nTachometer = %d\n', i);
    end 
    
    %disp(dataTacho(i+10));
end