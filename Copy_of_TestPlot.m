tbase = velData.T_Skm1_Sk;

pos2 = zeros(size(tbase,1),3);

tform = eye(4);

for i = 3:size(tbase,1)
    temp = V2T(tbase(i,:));
    temp(1:3,4) = temp(1:3,4);
    tform = tform*temp;
    pos2(i,:) = tform(1:3,4);
end

% pos2 = velData.T_S1_Sk(:,1:3);
% pos2 = [pos2(:,1:2),sqrt(1-pos2(:,1).^2 - pos2(:,2).^2),pos2(:,3)];
% pos2 = repmat(pos2(:,4),1,3).*pos2(:,1:3);

hold on;
plot3(pos2(:,1),pos2(:,2),pos2(:,3),'r-'); axis equal;