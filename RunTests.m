function [] = RunTests(testNum)

switch testNum
    case 1
        reps = 100;
        range = 10:5:200;
        timeSteps = 10000;
        
        parfor i = 1:9
            switch i
                case 1
                    %test with kitti and 0.1 second offset
                    [res,sensorData] = SetupTest1('Kitti', 4, reps, 0.1, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.1_Kitti_Time_0.1s');
                case 2 
                    %test with kitti and 1 second offset
                    [res,sensorData] = SetupTest1('Kitti', 4, reps, 1, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.2_Kitti_Time_1s');
                case 3
                    %test with kitti and 5 second offset
                    [res,sensorData] = SetupTest1('Kitti', 4, reps, 5, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.3_Kitti_Time_5s');
                case 4
                    %test with shrimp and 1 second offset
                    [res,sensorData] = SetupTest1('Shrimp', 5, reps, 0.1, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.4_Shrimp_Time_0.1s');
                case 5
                    %test with shrimp and 1 second offset
                    [res,sensorData] = SetupTest1('Shrimp', 5, reps, 1, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.5_Shrimp_Time_1s');
                case 6
                    %test with shrimp and 5 second offset
                    [res,sensorData] = SetupTest1('Shrimp', 5, reps, 5, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.6_Shrimp_Time_5s');
                case 7
                    %test with ford and 0.1 second offset
                    [res,sensorData] = SetupTest1('Ford', 5, reps, 0.1, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.7_Shrimp_Time_0.1s');
                case 8
                    %test with ford and 1 second offset
                    [res,sensorData] = SetupTest1('Ford', 5, reps, 1, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.8_Ford_Time_1s');
                case 9
                    %test with ford and 5 second offset
                    [res,sensorData] = SetupTest1('Ford', 5, reps, 5, range, timeSteps);
                    RunTest1(sensorData,res,'Test_1.9_Ford_Time_5s');
                otherwise
                    error('invalid num');
            end
        end
    case 2
        scansTimeRange = 120;
        numScans = 25;
        reps = 100;
        
        calib = cell(reps,1);
        dataset = 'Kitti';
        for i = 1:reps
            i
            calib{i} = Test2VelCam(scansTimeRange, numScans, dataset);
            ParSave(['./results/' 'Test_20.1_Kitti' '.mat'], calib, 'results');
        end
%         dataset = 'Shrimp';
%         for i = 1:reps
%             i
%             calib{i} = Test2VelCam(scansTimeRange, numScans, dataset);
%             ParSave(['./results/' 'Test_2.2_Shrimp' '.mat'], calib, 'results');
%         end
%         dataset = 'Ford';
%         for i = 1:reps
%             i
%             calib{i} = Test2VelCam(scansTimeRange, numScans, dataset);
%             ParSave(['./results/' 'Test_2.3_Ford' '.mat'], calib, 'results');
%         end
    case 3
        reps = 500;
        dataset = 'Kitti';
        calib = cell(500,1);
        for i = 1:reps
            i
            val = cell(30,1);
            parfor j = 1:30
                val{j} = Test3VelNav(10*j, dataset);
            end
            calib{i} = val;
            ParSave(['./results/' 'Test_3.10_Kitti' '.mat'], calib, 'results');
        end
        
        dataset = 'Shrimp';
        calib = cell(500,1);
        for i = 1:reps
            i
            val = cell(30,1);
            parfor j = 1:30
                val{j} = Test3VelNav(10*j, dataset);
            end
            calib{i} = val;
            ParSave(['./results/' 'Test_3.11_Shrimp' '.mat'], calib, 'results');
        end
        
    case 4
        reps = 500;
        dataset = 'Kitti';
        calib = cell(500,1);
        for i = 1:reps
            i
            val = cell(30,1);
            parfor j = 1:30
                val{j} = Test4NavCam(10*j, dataset);
            end
            calib{i} = val;
            ParSave(['./results/' 'Test_4.10_Kitti' '.mat'], calib, 'results');
        end
     case 5
        reps = 100;
        dataset = 'Kitti';
        calib = cell(100,1);
        for i = 1:reps
            i
            val = cell(30,1);
            parfor j = 1:30
                val{j} = Test5Multi(10*j, dataset);
            end
            calib{i} = val;
            ParSave(['./results/' 'Test_5.1_Kitti' '.mat'], calib, 'results');
        end
    otherwise
        error('Invalid test')
end
        

end

function [] = RunTest1(sensorData,res,saveName)
    for i = 1:res.Reps
        fprintf('Running iteration %i of %i\n',i,res.Reps);
        for j = 1:length(res.TimeLength)
            sData = RandTformTimes(sensorData, res.TimeLength(j));
            try
                [res.Error(:,i,j),res.Var(:,i,j)] = Test1CamTiming(sData, res.OffsetMag, res.TimeSamples);
            catch
                res.Error(:,i,j) = nan;
                res.Var(:,i,j) = nan;
            end
        end
        ParSave(['./results/' saveName '.mat'], res, 'results');
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
    res.Var = zeros(numCams,reps,length(timeLength));
    
    %format data string
    cams = cell(numCams,1);
    for i = 1:numCams
        cams{i} = ['Cam',num2str(i)];
    end

    %read camera data in
    sensorData = LoadSensorData(dataset,cams{:});
end

