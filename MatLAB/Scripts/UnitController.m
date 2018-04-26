classdef UnitController
    
    methods(Static)
        
        function [] = test1()
            disp("cake");
        end
        
        function [] = setDutyCycle(value)
            bmodule = evalin('base', 'bmodule');
            fwrite(bmodule, uint8(85));
            fwrite(bmodule, uint8(16));
            fwrite(bmodule, uint8(value));
        end
        
        function [] = setBroadcastMode(mode)
            bmodule = evalin('base', 'bmodule');
            fwrite(bmodule, uint8(85));
            fwrite(bmodule, uint8(20));
            fwrite(bmodule, uint8(mode));
            
            
            if (mode == 0)
                assignin('base', 'stateEnabled', false);
            else
                assignin('base', 'stateEnabled', true);
            end
            
        end
        
    end
 end