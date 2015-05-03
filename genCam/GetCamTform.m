function [T_Ckm1_Ck, T_Var_Ckm1_Ck] = GetCamTform(imOld, imNew, mask, K )
%GENCAMTFORM3 Gets normalized camera transform
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   imOld- n by m image, the previous image
%   imNew- n by m image, the current image
%   mask- n by m logical matrix. If zero points at this location will not
%   be used( allows removal of parts of vechile in view)
%   K- camera matrix
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   T_Ckm1_Ck - camera transformation vector from imOld to imNew.
%   T_Var_Ckm1_Ck - a vector giving the estimated variance of each element
%       of T_Ckm1_Ck
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

validateattributes(imOld,{'numeric'},{'2d'});
validateattributes(imNew,{'numeric'},{'2d','size',size(imOld)});
validateattributes(mask,{'logical'},{'2d','size',size(imOld)});
validateattributes(K,{'numeric'},{'size',[3,4]});

%detect features in first image
points = detectMinEigenFeatures(imOld);
points = points.Location;

%mask points
notMasked = mask(round(points(:,2))+size(mask,1)*(round(points(:,1))-1)) ~= 0;
points = points(notMasked,:);

%track points
pointTracker = vision.PointTracker('MaxBidirectionalError', 1);
initialize(pointTracker, points, imOld);
[pointsNew, matches] = step(pointTracker, imNew);
release(pointTracker);

%get matching points
points = points(matches,:);
pointsNew = pointsNew(matches,:);

%find transformation
[T_Ckm1_Ck, T_Var_Ckm1_Ck, ~, ~] = getTandPoints(pointsNew,points,K);

end

function [T_Ckm1_Ck, T_Var_Ckm1_Ck, points, inliers] = getTandPoints(mNew,mOld,K)

    %get fundemental matrix
    [F, inliers] = estimateFundamentalMatrix(mOld,mNew);%,'Method','MSAC','DistanceThreshold',0.01);
    mOld = mOld(inliers,:);
    mNew = mNew(inliers,:);
    
    %get essential matrix
    E = K(1:3,1:3)'*F*K(1:3,1:3);

    %Hartley matrices
    W = [0 -1 0; 1 0 0; 0 0 1];
       
    %get P
    [U,~,V] = svd(E);
    RA = U*W*V';
    RB = U*W'*V';
    
    RA = RA*sign(RA(1,1));
    RB = RB*sign(RB(1,1));
    
    TA = U(:,3);
    TB = -U(:,3);
    
    %get Transform
    T_Ckm1_Ck = zeros(4,4,4);
    
    T_Ckm1_Ck(:,:,1) = [[RA,TA];[0,0,0,1]];
    T_Ckm1_Ck(:,:,2) = [[RA,TB];[0,0,0,1]];
    T_Ckm1_Ck(:,:,3) = [[RB,TA];[0,0,0,1]];
    T_Ckm1_Ck(:,:,4) = [[RB,TB];[0,0,0,1]];
        
    best = zeros(4,1);
 
    %match points
    points = zeros(size(mNew,1),6,4);
    for j = 1:4
        T_Ckm1_Ck(:,:,j) = T_Ckm1_Ck(:,:,j);
        points(:,:,j) = getPoints(mNew,mOld,T_Ckm1_Ck(:,:,j),K);
        best(j,1) = median(points(:,3,j)) + median(points(:,6,j));
    end
    
    [~,idx] = max(best);
    points = points(:,:,idx);
    T_Ckm1_Ck = T_Ckm1_Ck(:,:,idx);
    
    %estimate variance
    T_Ckm1_Ck = T2V(T_Ckm1_Ck);
    T_Var_Ckm1_Ck = FindCamVar(T_Ckm1_Ck, mNew, mOld, K);
    
%     %sample data
%     T_Var_Ckm1_Ck = zeros(100,6);
%     for i = 1:100
%         %get sampled points
%         [mOBS,idx] = datasample(mOld,size(mOld,1));
%         mNBS = mNew(idx,:);
% 
%         %find fundemental matrix (using fundmatrix for speed)
%         F = Fundmatrix([mOBS,ones(size(mOBS,1),1)]',[mNBS,ones(size(mOBS,1),1)]');
% 
%         %add random error to focal length and centre point
%         Kr = K + [randn(1),0,randn(1),0;0,randn(1),randn(1),0;0,0,0,0];
% 
%         %get essential matrix
%         E = Kr(1:3,1:3)'*F*Kr(1:3,1:3);
% 
%         %get P
%         [U,~,V] = svd(E);
%         RA = U*W*V';
%         RB = U*W'*V';
% 
%         if (sum(U(:,3)) > 0)
%             T = U(:,3);
%         else
%            T = -U(:,3);
%         end
% 
%         RA = repmat(sign(diag(RA)),1,3).*RA;
%         RB = repmat(sign(diag(RB)),1,3).*RB;
% 
%         eA = RA - T_Ckm1_Ck(1:3,1:3); eA = sum(eA(:).^2);
%         eB = RB - T_Ckm1_Ck(1:3,1:3); eB = sum(eB(:).^2);
% 
%         if (eB > eA)
%             T_Var_Ckm1_Ck(i,:) = T2V([RA,T;[0,0,0,1]]);
%         else
%             T_Var_Ckm1_Ck(i,:) = T2V([RB,T;[0,0,0,1]]);
%         end
%     end
%     T_Var_Ckm1_Ck = var(T_Var_Ckm1_Ck);
%     T_Ckm1_Ck = T2V(T_Ckm1_Ck);
    
    %filter out negitive and distant point matches
    badPoints = or(sqrt(sum(points.^2,2)) > 1000, points(:,3) < 0);
    in = find(inliers);
    inliers(in(badPoints)) = 0;
    points = points(~badPoints,:);
end

function [points] = getPoints(mNew,mOld,T,K)
    
    %match points
    points = zeros(size(mNew,1),6);
    
    P1 = K;
    P2 = K*T;
    for i = 1:size(mNew,1)

        A = zeros(4,4);
        A(1,:) = mOld(i,1)*P1(3,:)' - P1(1,:)';
        A(2,:) = mOld(i,2)*P1(3,:)' - P1(2,:)';
        A(3,:) = mNew(i,1)*P2(3,:)' - P2(1,:)';
        A(4,:) = mNew(i,2)*P2(3,:)' - P2(2,:)';

        [~, ~, V] = svd(A);
        points(i,1:3) = V(1:3,4)'/V(4,4);
        temp = T*[points(i,1:3),1]';
        points(i,4:6) = temp(1:3);
    end
end

