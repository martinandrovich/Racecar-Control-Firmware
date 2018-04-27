function [newPlotTach,newPlotAcc] = mappingOffSet(tachometer,accelerometer)

if ~isvector(tachometer) || ~isvector(accelerometer)
    disp('Error = is not a vector')
    return
end

if length(tachometer) ~= length(accelerometer)
    disp('Error = length is not equal')
    return
end

i = 1;
j = 1;
k = 2;
offSetArray = [ 9 , 10 ];

newPlotAcc = zeros(1,length(accelerometer));
newPlotTach = zeros(1,length(tachometer));

while i < (length(accelerometer) - 1)
    
    if accelerometer(i) ~= accelerometer(i+1)
        
        newPlotAcc(k) = accelerometer(i);
        newPlotTach(k) = tachometer(i) - offSetArray(j);
        
        k = k + 1;
        
        newPlotAcc(k) = accelerometer(i+1) ;
        newPlotTach(k) = tachometer(i+1) - offSetArray(j);
        
        k = k + 1;
        
        j = j + 1;
        
        if (j == 3)
            j = 1;
        end   
        
    end
    i = i + 1;
end
% 
% newPlotAcc(k:length(accelerometer)) = [];
% newPlotTach(k:length(tachometer)) = [];
% clf;
% plot(tachometer,accelerometer);
% hold on;
% plot(newPlotTach,newPlotAcc);

end