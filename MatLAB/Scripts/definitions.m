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
                    'Velocity',         56,     ...
                    'Accelerometer',    40      ...
                  );           


% Circuit OUTER = 290
% Circuit INNER = 266

logDuration     = 10;
logDistance     = 290;
logLaps         = 1;
movingAvgSize   = 32;
dutyCycle       = 90;
speedBrake      = 60;
speedAccelerate = 120;
offsetVal       = 10;
bufferDelay     = 0.5;

prealloc        = 4096;
data            = eye(prealloc, 1);
dataLong        = uint16(0);
dataAcclr       = uint8(0);
dataTacho       = uint16(0);
dataVelocity    = uint16(0);
dataMapping     = 0;
dataTrajectory  = 0;
count           = 1;
tachoVal        = 0;
stateEnabled    = false;
boolTurn        = false;

fprintf('Definitions OK.\n\n');

%