% Global definitions
run('definitions.m');

% Tachometer & Accelerometer Logging
disp("Internal Mapping [Turn/Ticks]");
disp("Version 1.0.0");

% Plot configuration
plotTitle       = 'Mapping Plot';
xLabel          = 'Ticks';
yLabel          = 'Turn Boolean Value';
yMax            =  2;
yMin            =  0;
plotGrid        = 'on';

% Connect to Bluetooth Module
fopen(bmodule);
fprintf('\nConnection established; starting data logging.');

% Start mapping
UnitController.enableMapping();

% Await input
input('\nPress ENTER when Mapping is complete.');

% Read mapping
UnitController.getMapping();

while (true)
    
    if (bmodule.BytesAvailable >= 2)
        
        dataBytes = fread(bmodule, 2);
        dataLong = bitor(bitshift(dataBytes(1), 8), dataBytes(2));
        
        % Exit if EoT
        if (dataLong == 65535)
            break;
        end
        
        % Extract turn bit
        dataMapping(count, 2) = bitget(dataLong, 16);
        
        % Extract Tachometer value
        dataMapping(count, 1) = bitset(dataLong, 16, 0);
        
        count = count + 1;
        
    end
    
end

% Plot mapping
plotGraph = plot(dataMapping(:,1), dataMapping(:,2), ':bs', ...
            'MarkerSize', 8, 'MarkerFaceColor', 'r');
hold on;
title(plotTitle, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 15);
ylabel(yLabel, 'FontSize', 15);
axis([0 inf yMin yMax]);
grid(plotGrid);

% Create & Plot square mapping
x=dataMapping(:,1);
y=dataMapping(:,2);
a=x([2:end,end]);
a=reshape([x,a]',[],1);
b=reshape([y,y]',[],1);
plot(a,b,'r-');

% Maximize figure window
drawnow;
set(get(handle(gcf), 'JavaFrame'), 'Maximized', 1);

% Close connection
fclose(bmodule);