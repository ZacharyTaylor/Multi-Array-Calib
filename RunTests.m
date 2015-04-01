function [] = RunTests(testNum)

%     %test with kitti and 1 second offset
%     [res,sensorData] = SetupTest1('Kitti', 4, 100, 1, 10:10:200, 10000);
%     for i = 1:res.Reps
%         fprintf('Running iteration %i of %i\n',i,res.Reps);
%         for j = 1:length(res.TimeLength)
%             sData = RandTformTimes(sensorData, res.TimeLength(j));
%             res.Error(:,i,j) = Test1CamTiming(sData, res.OffsetMag, res.TimeSamples);
%         end
%         ParSave(['./results/' 'Test_1.1_Kitti_Time_1s' '.mat'], res, 'results');
%     end
%     
%     %test with kitti and 5 second offset
%     [res,sensorData] = SetupTest1('Kitti', 4, 100, 5, 10:10:200, 10000);
%     for i = 1:res.Reps
%         fprintf('Running iteration %i of %i\n',i,res.Reps);
%         for j = 1:length(res.TimeLength)
%             sData = RandTformTimes(sensorData, res.TimeLength(j));
%             res.Error(:,i,j) = Test1CamTiming(sData, res.OffsetMag, res.TimeSamples);
%         end
%         ParSave(['./results/' 'Test_1.2_Kitti_Time_5s' '.mat'], res, 'results');
%     end
%     
%     %test with shrimp and 1 second offset
%     [res,sensorData] = SetupTest1('Shrimp', 5, 100, 1, 10:10:200, 10000);
%     for i = 1:res.Reps
%         fprintf('Running iteration %i of %i\n',i,res.Reps);
%         for j = 1:length(res.TimeLength)
%             sData = RandTformTimes(sensorData, res.TimeLength(j));
%             res.Error(:,i,j) = Test1CamTiming(sData, res.OffsetMag, res.TimeSamples);
%         end
%         ParSave(['./results/' 'Test_1.3_Shrimp_Time_1s' '.mat'], res, 'results');
%     end
%     
%     %test with shrimp and 5 second offset
%     [res,sensorData] = SetupTest1('Shrimp', 5, 100, 5, 10:10:200, 10000);
%     for i = 1:res.Reps
%         fprintf('Running iteration %i of %i\n',i,res.Reps);
%         for j = 1:length(res.TimeLength)
%             sData = RandTformTimes(sensorData, res.TimeLength(j));
%             res.Error(:,i,j) = Test1CamTiming(sData, res.OffsetMag, res.TimeSamples);
%         end
%         ParSave(['./results/' 'Test_1.4_Shrimp_Time_5s' '.mat'], res, 'results');
%     end
    
    %test with ford and 1 second offset
    [res,sensorData] = SetupTest1('Ford', 5, 100, 1, 10:10:200, 10000);
    for i = 1:res.Reps
        fprintf('Running iteration %i of %i\n',i,res.Reps);
        for j = 1:length(res.TimeLength)
            sData = RandTformTimes(sensorData, res.TimeLength(j));
            res.Error(:,i,j) = Test1CamTiming(sData, res.OffsetMag, res.TimeSamples);
        end
        ParSave(['./results/' 'Test_1.5_Ford_Time_1s' '.mat'], res, 'results');
    end
    
    %test with ford and 5 second offset
    [res,sensorData] = SetupTest1('Ford', 5, 100, 5, 10:10:200, 10000);
    for i = 1:res.Reps
        fprintf('Running iteration %i of %i\n',i,res.Reps);
        for j = 1:length(res.TimeLength)
            sData = RandTformTimes(sensorData, res.TimeLength(j));
            res.Error(:,i,j) = Test1CamTiming(sData, res.OffsetMag, res.TimeSamples);
        end
        ParSave(['./results/' 'Test_1.6_Ford_Time_5s' '.mat'], res, 'results');
    end

end

function [res, sensorData] = SetupTest1(dataset, numCams, reps, offsetMag, timeLength, timeSamples)
    
    res = struct;
    res.Test = 'Test 1 Timing Offset';
    res.TimeLength = timeLength;
    res.OffsetMag = offsetMag;
    res.Reps = reps;
    res.Datasets = dataset;
    res.NumCams = numCams;
    res.TimeSamples = timeSamples;
    res.Error = zeros(numCams,reps,length(timeLength));
    
    %format data string
    cams = cell(numCams,1);
    for i = 1:numCams
        cams{i} = ['Cam',num2str(i)];
    end

    %read camera data in
    sensorData = LoadSensorData(dataset,cams{:});
end

