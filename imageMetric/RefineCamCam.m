function [ T, T_Var ] = RefineCamCam( camA, camB, numImages )

dataIdx = datasample(1:size(camA.files,1),numImages,'Replace',false);

pointsA = cell(numImages,1);
pointsB = cell(numImages,1);

for i = 1:numImages
    
    %get image from A
    image = imread([camA.folder camA.files(dataIdx(i)).name]);
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end
    image = uint8(Undistort(double(image), camA.D, camA.K));
    surfA = detectSURFFeatures(image);
    [featA, surfA] = extractFeatures(image, surfA);

    %find closest image from B
    time = double(camB.time) - double(camA.time(dataIdx(i)));
    [~,idx] = min(abs(time));
    
    %get image from B
    image = imread([camB.folder camB.files(idx).name]);
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end
    image = uint8(Undistort(double(image), camB.D, camB.K));
    surfB = detectSURFFeatures(image);
    [featB, surfB] = extractFeatures(image, surfB);
    
    %match features
    matches = matchFeatures(featA, featB, 'Prenormalized', true) ;

    pointsA{i} = surfA.Location(matches(:, 1),:);
    pointsB{i} = surfB.Location(matches(:, 2),:);
end

%convert to list
pointsA = cell2mat(pointsA);
pointsB = cell2mat(pointsB);

%get transformation and variance
[T, T_Var] = getTandPoints(pointsA, pointsB, camA.K, camB.K);
end

function [T_Ckm1_Ck, T_Var_Ckm1_Ck, points, inliers] = getTandPoints(pointsA,pointsB,KA,KB)

    %get fundemental matrix
    [F, inliers] = estimateFundamentalMatrix(pointsB,pointsA);%,'Method','MSAC','DistanceThreshold',0.01);
    pointsB = pointsB(inliers,:);
    pointsA = pointsA(inliers,:);
    
    %get essential matrix
    E = KB(1:3,1:3)'*F*KA(1:3,1:3);

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
    points = zeros(size(pointsA,1),6,4);
    for j = 1:4
        T_Ckm1_Ck(:,:,j) = T_Ckm1_Ck(:,:,j);
        points(:,:,j) = getPoints(pointsA,pointsB,T_Ckm1_Ck(:,:,j),KA,KB);
        best(j,1) = median(points(:,3,j)) + median(points(:,6,j));
    end
    
    [~,idx] = max(best);
    points = points(:,:,idx);
    T_Ckm1_Ck = T_Ckm1_Ck(:,:,idx);
    
    %sample data
    T_Var_Ckm1_Ck = zeros(100,6);
    for i = 1:100
        %get sampled points
        [mOBS,idx] = datasample(pointsB,size(pointsB,1));
        mNBS = pointsA(idx,:);
        
        %find fundemental matrix (using fundmatrix for speed)
        F = Fundmatrix([mOBS,ones(size(mOBS,1),1)]',[mNBS,ones(size(mOBS,1),1)]');
        
        %add random error to focal length and centre point
        KAr = KA + [randn(1),0,randn(1),0;0,randn(1),randn(1),0;0,0,0,0];
        KBr = KB + [randn(1),0,randn(1),0;0,randn(1),randn(1),0;0,0,0,0];
        
        %get essential matrix
        E = KBr(1:3,1:3)'*F*KAr(1:3,1:3);
       
        %get P
        [U,~,V] = svd(E);
        RA = U*W*V';
        RB = U*W'*V';

        if (sum(U(:,3)) > 0)
            T = U(:,3);
        else
           T = -U(:,3);
        end
        
        RA = repmat(sign(diag(RA)),1,3).*RA;
        RB = repmat(sign(diag(RB)),1,3).*RB;
        
        eA = RA - T_Ckm1_Ck(1:3,1:3); eA = sum(eA(:).^2);
        eB = RB - T_Ckm1_Ck(1:3,1:3); eB = sum(eB(:).^2);
        
        if (eB > eA)
            T_Var_Ckm1_Ck(i,:) = T2V([RA,T;[0,0,0,1]]);
        else
            T_Var_Ckm1_Ck(i,:) = T2V([RB,T;[0,0,0,1]]);
        end
    end
    T_Var_Ckm1_Ck = var(T_Var_Ckm1_Ck);
    
    %filter out negitive and distant point matches
    badPoints = or(sqrt(sum(points.^2,2)) > 1000, points(:,3) < 0);
    in = find(inliers);
    inliers(in(badPoints)) = 0;
    points = points(~badPoints,:);
    
    T_Ckm1_Ck = T2V(T_Ckm1_Ck);
end

function [points] = getPoints(pointsA,pointsB,T,KA,KB)
    
    %match points
    points = zeros(size(pointsA,1),6);
    
    P1 = KA;
    P2 = KB*T;
    for i = 1:size(pointsA,1)

        A = zeros(4,4);
        A(1,:) = pointsB(i,1)*P1(3,:)' - P1(1,:)';
        A(2,:) = pointsB(i,2)*P1(3,:)' - P1(2,:)';
        A(3,:) = pointsA(i,1)*P2(3,:)' - P2(1,:)';
        A(4,:) = pointsA(i,2)*P2(3,:)' - P2(2,:)';

        [~, ~, V] = svd(A);
        points(i,1:3) = V(1:3,4)'/V(4,4);
        temp = T*[points(i,1:3),1]';
        points(i,4:6) = temp(1:3);
    end
end

