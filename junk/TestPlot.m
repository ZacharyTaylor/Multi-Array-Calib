tbase = cam1Data.T_Skm1_Sk;

scale = ones(size(tbase,1),1);
pos2 = zeros(size(tbase,1),3);

tform = eye(4);

for i = 3:size(tbase,1)
    temp = V2T(tbase(i,:));
    
    %temp(1:3,4) = temp(1:3,4)./norm(temp(1:3,4));
    %temp(1:3,4) = scale(i-1)*temp(1:3,4);

    tform = tform*temp;
    pos2(i,:) = tform(1:3,4);
    %scale(i) = norm(temp(1:3,4));
   
end

hold on;
plot3(pos2(:,1),pos2(:,2),pos2(:,3),'b-'); axis equal;