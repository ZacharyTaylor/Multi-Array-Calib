function [ tGrid, vGrid ] = GenTformGrid( tranVec, rotVec, tranVar, rotVar )
%GENGRID generates grid of all possible transformations and rotations
%   between sensors with variance estimates. Uses monty-carlo to transfer
%   variance
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   tranVec- nx3 matrix of translations for each sensor
%   rotVec- nx3 matrix of rotations for each sensor
%   tranVar- nx3 matrix of translation variances for each sensor
%   rotVar- nx3 matrix of rotation variances for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   tGrid- nxn cell containing every possible transformation matrix
%       between the sensors
%   vGrid- nxn cell containing every possible variance vector between 
%       the sensors
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

validateattributes(tranVec,{'numeric'},{'ncols',3});
validateattributes(rotVec,{'numeric'},{'size',[size(tranVec,1),3]});
validateattributes(tranVar,{'numeric'},{'size',[size(tranVec,1),3]});
validateattributes(rotVar,{'numeric'},{'size',[size(tranVec,1),3]});

%number of times to run variance estimate
SampleNum = 100;

%size of the grid
gs = size(tranVec,1);

%preallocate memory
tGrid = cell(gs);
vGrid = cell(gs);

for i = 1:gs
    for j = 1:gs
        %only calculate half of the transforms
        if(i >= j)
            tGrid{i,j} = zeros(6,1);
            vGrid{i,j} = zeros(6,1);
        else
            %find transform
            tGrid{i,j} = T2V(V2T([tranVec(j,:), rotVec(j,:)])/V2T([tranVec(i,:), rotVec(i,:)]));
            
            %find variance through random sampling
            res = zeros(SampleNum,6);
            a = mvnrnd([tranVec(i,:), rotVec(i,:)],diag([tranVar(i,:), rotVar(i,:)]),SampleNum);
            b = mvnrnd([tranVec(j,:), rotVec(j,:)],diag([tranVar(j,:), rotVar(j,:)]),SampleNum);

            for k = 1:SampleNum
                Tab = V2T(b(k,:))/V2T(a(k,:));
                res(k,:) = T2V(Tab);
            end

            vGrid{i,j} = var(res);
        end
    end
end

end

