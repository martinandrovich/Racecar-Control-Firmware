classdef UnitController
    
    methods(Static)
        
        function [] = setDutyCycle(value)
            bmodule = evalin('base', 'bmodule');
            fwrite(bmodule, uint8(85));
            fwrite(bmodule, uint8(16));
            fwrite(bmodule, uint8(value));
        end
        
        function [] = setBroadcastMode(mode)
            
            if (mode == 0)
                assignin('base', 'stateEnabled', false);
            else
                assignin('base', 'stateEnabled', true);
            end
            
            bmodule = evalin('base', 'bmodule');
            fwrite(bmodule, uint8(85));
            fwrite(bmodule, uint8(20));
            fwrite(bmodule, uint8(mode)); 
        end
        
        function [] = enableMapping()
            bmodule = evalin('base', 'bmodule');
            fwrite(bmodule, uint8(85));
            fwrite(bmodule, uint8(19));
            fwrite(bmodule, uint8(0));
        end
        
        function [] = getMapping()
            bmodule = evalin('base', 'bmodule');
            fwrite(bmodule, uint8(170));
            fwrite(bmodule, uint8(19));
            fwrite(bmodule, uint8(0));
        end

        function [] = getTrajectory()
            bmodule = evalin('base', 'bmodule');
            fwrite(bmodule, uint8(170));
            fwrite(bmodule, uint8(22));
            fwrite(bmodule, uint8(0));
        end
        
    end
 end