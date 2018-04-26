% Clear everything
clear;
clc;
clf;
close(gcf);

% Definitions
bmodule = Bluetooth('RNBT-E2A9', 1);

broadcastModes  = struct(...
                    'Disabled',         0,      ...                    
                    'All',              8,      ...
                    'Tachometer',       24,     ...
                    'Finishline',       56,     ...
                    'Accelerometer',    40      ...
                  );           

logDuration     = 10;
logDistance     = 290;
movingAvgSize   = 32;
dutyCycle       = 70;
stateEnabled    = false;

data            = 0;
dataLong        = uint16(0);
dataAcclr       = uint8(0);
dataTacho       = uint16(0);
count           = 1;
tachoVal        = 0;

fprintf('Definitions OK.\n\n');