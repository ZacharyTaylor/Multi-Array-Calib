function [ SensorInt ] = matchTformsTime( Sensor, times )
%MATCHTFORMS interpolates sensor A's transforms to match the times sensor
%B's occoured at.

%setup ouput
SensorInt = Sensor;
SensorInt.T_S1_Sk(length(times),6) = 0;
SensorInt.T_S1_Sk = SensorInt.T_S1_Sk(1:length(times),1:6);

SensorInt.T_Skm1_Sk(length(times),6) = 0;
SensorInt.T_Skm1_Sk = SensorInt.T_Skm1_Sk(1:length(times),1:6);

SensorInt.T_Cov_Skm1_Sk(length(times),6) = 0;
SensorInt.T_Cov_Skm1_Sk = SensorInt.T_Cov_Skm1_Sk(1:length(times),1:6);

SensorInt.time(length(times),1) = 0;
SensorInt.time = SensorInt.time(1:length(times),1);

SensorInt.files(length(times),1) = Sensor.files(1);
SensorInt.files = SensorInt.files(1:length(times),1);

%add error
tempCov = Sensor.T_Cov_Skm1_Sk;
if(~strcmp(Sensor.type,'nav'));
    for i = 1:size(Sensor.T_Cov_Skm1_Sk,3)
        if(any(tempCov(i,:) > 100))
            tempCov(i,:) = 100*ones(6,1);
        end
        if(i < 2)
            continue;
        end
        tempCov(i,:) = tempCov(i-1,:) + tempCov(i,:);
    end
end

for i = 1:size(times(:),1)
    
    %find closest matching time
    tdiff = double(Sensor.time) - double(times(i));

    %get matching tform
    [~,idx] = min(abs(tdiff));
    tdiff = tdiff(idx);
    
    %get base transform
    tform = vec2tran(Sensor.T_S1_Sk(idx,:)');

    %get other elements
    SensorInt.time(i) = times(i);
    if(size(Sensor.files,1) > 1)
        SensorInt.files(i) = Sensor.files(idx);
    end
    
    %get covariance
    SensorInt.T_Cov_Skm1_Sk(i,:) = tempCov(idx,:);
    
    %get fractional transform
    if(idx > 1)
        %get closest relative tform
        if(tdiff > 0)
            tformF = vec2tran(Sensor.T_Skm1_Sk(idx,:)');
        else
           tformF = vec2tran(Sensor.T_Skm1_Sk(idx-1,:)');
        end
        
        %get time difference
        tratio = tdiff/(double(Sensor.time(idx))-double(Sensor.time(idx-1)));

        %interpolate
        tformF = tformInterp(tformF,-tratio);

        tform = tform*tformF;
    end
    
    SensorInt.T_S1_Sk(i,:) = tran2vec(tform)';
end

for i = 2:size(SensorInt.T_S1_Sk,1)
    SensorInt.T_Skm1_Sk(i,:) = tran2vec(vec2tran(SensorInt.T_S1_Sk(i-1,:)')\vec2tran(SensorInt.T_S1_Sk(i,:)'))';
end

if(~strcmp(Sensor.type,'nav'))
    for i = size(SensorInt.T_Cov_Skm1_Sk,3):-1:2
        SensorInt.T_Cov_Skm1_Sk(i,:) = SensorInt.T_Cov_Skm1_Sk(i,:) - SensorInt.T_Cov_Skm1_Sk(i-1,:);
    end
end
    