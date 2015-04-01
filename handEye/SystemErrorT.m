function [ err, verr ] = SystemErrorT( TData, vTData, s, estVec, rotVec, rotVar )
%SYSTEMPROBT estimate error from variance in trans vectors after alignment
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   tformMat- nx1 cell of tform mats in a vector format (see OptT)
%   estVec- (n-1)x3 matrix of translations for each sensor (1st sensor info
%       not given as it will just be 0,0,0)
%   rotMat- 3x3xn matrix of rotation matrix for each sensor (1st sensor
%       info not given as it will just be 0,0,0)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   prob- measure of system likelihood (note not an actual probabilty)
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

%check inputs
validateattributes(TData,{'double'},{'3d','ncols',6});
validateattributes(vTData,{'double'},{'size' size(TData)});
validateattributes(s,{'double'},{'vector','numel',size(TData,3)});
validateattributes(estVec,{'numeric'},{'size',[size(TData,3)-1,3]});
%validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%set first element to zeros
estVec = [0,0,0;estVec];

%find probablity of system
err = zeros(size(TData(:,:,1)))';
verr = zeros(size(TData(:,:,1)))';
for a = 1:length(s)
    for b = 1:length(s)
        if(a < b)
            %skip camera-camera matching (just too hard to be worth it)
            if(s(a))            
                continue;
            end
            
            R = [rotVec(a,:); rotVec(b,:)]';
            vR = [rotVar(a,:); rotVar(b,:)]';
            
            t = V2R(rotVec(a,:))'*(estVec(b,:) - estVec(a,:))';
            
            tA = TData(:,1:3,a)';
            vtA = vTData(:,1:3,a)';
            tB = TData(:,1:3,b)';
            vtB = vTData(:,1:3,b)';
            
            RA = TData(:,4:6,a)';
            vRA = vTData(:,4:6,a)';
            RB = TData(:,4:6,b)';
            vRB = vTData(:,4:6,b)';
                     
            [temp,vtemp] = errT(R,vR,tA,vtA,RA,vRA,tB,vtB,RB,vRB,t,s(b));
            
            err = err + temp;
            verr = verr + vtemp;
        end
    end
end

err = err';
verr = verr';

end

