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
%bootNum = 100;

%samples
timeSamples = 100000;
samples = 2000;

%% load sensor data
sensorData = LoadSensorData('Kitti', 'Vel', 'Nav', 'Cam1', 'Cam2');

%% fix timestamps
[sensorData, offsets] = CorrectTimestamps(sensorData, timeSamples);

%% evenly sample data
sensorData = SampleData(sensorData, samples);

%% find transformations

RErr = zeros(reps,3,size(scansTimeRange(:),1));
TErr = zeros(reps,3,size(scansTimeRange(:),1));

RVar = zeros(reps,3,size(scansTimeRange(:),1));
TVar = zeros(reps,3,size(scansTimeRange(:),1));

RErrEqual = zeros(reps,3,size(scansTimeRange(:),1));
TErrEqual = zeros(reps,3,size(scansTimeRange(:),1));

for w = 1:reps
    for s = 1:size(scansTimeRange(:),1)
        
        %get random contiguous scans to use
        sData = RandTformTimes(sensorData, scansTimeRange(s));

        %Create equal weighted variance
        sDataE = sData;
        for i = 1:size(sData,1)
            sDataE{i}.T_Var_Skm1_Sk = ones(size(sData{1}.T_Var_Skm1_Sk));
        end
        
        %find equal weighted results
        rotVec = RoughR(sDataE);
        tranVec = RoughT(sDataE, rotVec);

        %write out results
        RErrEqual(w,:,s) = rotVec(2,:);
        TErrEqual(w,:,s) = tranVec(2,:);

        %remove uninformative data
        sData = RejectPoints(sData, 3, 0.001);
        
        %find rotation
        rotVec = RoughR(sData);
        rotVec = OptR(sData, rotVec);
        rotVar = ErrorEstR(sensorData, rotVec);
        
        %find translation
        tranVec = RoughT(sData, rotVec);
        tranVec = OptT(sData, tranVec, rotVec);
        tranVar = ErrorEstT(sData, tranVec, rotVec);

        %bootstrap
        %[tranVar, rotVar] = bootTform(sData, tranVec, rotVec, bootNum);

        %write out results
        RErr(w,:,s) = rotVec(2,:);
        TErr(w,:,s) = tranVec(2,:);
        RVar(w,:,s) = rotVar(2,:);
        TVar(w,:,s) = tranVar(2,:);

        fprintf('R = [% 1.3f,% 1.3f,% 1.3f], T = [% 3.2f,% 3.2f,% 3.2f] using %4i seconds of data, iteration = %i\n',rotVec(2,1),rotVec(2,2),rotVec(2,3),tranVec(2,1),tranVec(2,2),tranVec(2,3),scansTimeRange(s),w);

        save('Test_1_Res.mat', 'RErr', 'TErr', 'RVar', 'TVar', 'RErrEqual', 'TErrEqual');
    end
end
