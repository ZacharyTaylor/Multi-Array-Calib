function [ prob ] = SystemProbT( TData, vTData, s, estVec, rotVec, rotVar, covFlag )
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
prob = zeros(length(s));
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
                     
            [e,v] = logpdfT2(R,vR,tA,vtA,RA,vRA,tB,vtB,RB,vRB,t,s(b));
            e = e';
            v = v';
            
            w = zeros(6,1);
            for i = 1:6
                temp = xcorr(e(:,i),1);
                w(i) = 1 - abs(temp(1)/temp(2));
            end
            
            w = sqrt(prod(w));
            %prod(w)
            
                        
            eExp1 = -0.5*(e(:,1).*e(:,1)./v(:,1) + e(:,2).*e(:,2)./v(:,2) + e(:,3).*e(:,3)./v(:,3));
            bExp1 = -log(sqrt(8*pi*pi*pi.*v(:,1).*v(:,2).*v(:,3)));

            eExp2 = -0.5*(e(:,4).*e(:,4)./v(:,4) + e(:,5).*e(:,5)./v(:,5) + e(:,6).*e(:,6)./v(:,6));
            bExp2 = -log(sqrt(8*pi*pi*pi.*v(:,4).*v(:,5).*v(:,6)));

            temp = (bExp1 + eExp1 + bExp2 + eExp2)/2;
            
            temp = sort(temp,'descend');
            temp = temp(1:floor(size(temp,1)*0.5));
            temp = -sum(temp);
            
            prob(a,b) = temp;
        end
    end
end

prob = mean(prob(:));

%add zero bias
%prob = prob + 1000*sum(estVec(:).^2);

end

