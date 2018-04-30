offset = 10;
faketacho = 1;

while faketacho < 55
    
    tachoVal = faketacho + offset;

    if(find(datarr(:,1) == tachoVal))
        
        switch datarr(find(datarr(:,1) == tachoVal), 2)
            case 0
                fprintf('Accelerate at: %d\n\n', tachoVal);
            case 1
                fprintf('Break at: %d\n\n', tachoVal);
        end
        
    end
    
    faketacho = faketacho +1;
    
end