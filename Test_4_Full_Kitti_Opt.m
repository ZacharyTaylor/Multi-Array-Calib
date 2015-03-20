% this script generates the first set of results in the paper
% It calculates the transformation between the velodyne and camera 0 for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

%% user set variables

%number of scans to use
scansTimeRange = 100;
%scansTimeRange = 5:5:100;

%number of times to perform test
reps = 10;

%samples
timeSamples = 100000;
samples = 5000;


%% load sensor data
CalibPath(true);
sensorData = LoadSensorData('Shrimp','Vel', 'Cam1');

sensorData = InvertSensorData(sensorData);

%% fix timestamps
[sensorData, offsets] = CorrectTimestamps(sensorData, timeSamples);

%% correct camera scale
    
outT = cell(100,1);
outV = cell(100,1);

outTB = cell(100,1);
outVB = cell(100,1);

for w = 1:reps
    tic
    
    %get random contiguous scans to use
    sDataBase = RandTformTimes(sensorData, scansTimeRange);
    
    %evenly sample data
    sData = SampleData(sDataBase, samples);
    
    %remove uninformative data
    sData = RejectPoints(sData, 10, 0.0001);

    %find rotation
    fprintf('Finding Rotation\n');
    rotVec = RoughR(sData);
    rotVec = OptR(sData, rotVec);
    rotVar = ErrorEstCR(sData, rotVec,0.01);
    %[ outVec ] = ErrorEstWR(sData,rotVec);
    %rotVar2 = ErrorEstR(sData, rotVec,100);
    
    %find camera transformation scale
 %   fprintf('Finding Camera Scale\n');
    sData = EasyScale(sData, rotVec, rotVar,zeros(2,3),ones(2,3));
    
    %clean up large variance
    sData = ThresholdVar(sData,10);
    
    %show what we are dealing with
    PlotData(sData,rotVec);

    %find translation
    %sData = TDiff(sData, rotVec, rotVar);
    
    fprintf('Finding Translation\n');
    tranVec = RoughT(sData, rotVec);
    tranVec = OptT(sData, tranVec, rotVec, rotVar);
    tranVar = ErrorEstCT(sData, tranVec, rotVec, rotVar, 0.01);
    %tranVar2 = ErrorEstT(sData, tranVec, rotVec, rotVar2, 100);

    %get grid of transforms
    fprintf('Generating transformation grid\n');
    [tGrid, vGrid] = GenTformGrid(tranVec, rotVec, tranVar, rotVar);
    
    %refine transforms using metrics
    fprintf('Refining transformations\n');
    [tGridR, vGridR] = metricRefine(tGrid, vGrid, sDataBase,0.1,10);
    
    %correct for differences in grid
    fprintf('Combining results\n');
    [tVals, vVals] = optGrid(tGridR, vGridR, sensorData);
    
    outT{w} = tVals;
    outV{w} = vVals;
    
    outTB{w} = [tranVec, rotVec];
    outVB{w} = [tranVar, rotVar];
    
    save('Test_45_Res.mat','outT','outV','outTB','outVB');
    toc
    w
end
