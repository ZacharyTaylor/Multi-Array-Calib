function [ Tvel, varTvel ] = getTvel( vel1, vel2, Told )
%GETTCAM Gets normalized camera transform given two images and K
% optionally also generates estimated variance in second output argument
     
%get points less then 40 meters away
dist = sqrt(sum(vel1(:,1:3).^2,2));
vel1 = vel1(dist < 40,:);

dist = sqrt(sum(vel2(:,1:3).^2,2));
vel2 = vel2(dist < 40,:);

t1 = getTime( vel1, 0); 
t2 = getTime( vel2, 0); 

Told = vec2tran(Told');
v2 = tformInterp3(inv(Told),t2,vel2(:,1:3));
%v1 = tformInterp3(Told,t1,vel1(:,1:3));
v1 = vel1(:,1:3);
Tvel = icpMex(v2',v1',Told,-t1',1,'point_to_plane');
Tvel = tran2vec(Tvel)';

s = min(size(v1,1),size(v2,1));
vel1 = v1(1:s,:);
vel2 = v2(1:s,:);

varTvel = zeros(100,6);
for i = 1:100
    [s1,idx] = datasample(vel1(:,1:3),5000);
    s2 = vel2(idx,1:3);
    varTvel(i,:) = tran2vec(icpMex(s2',s1',vec2tran(Tvel),-t1(idx)',1,'point_to_point'))';
end

varTvel = var(varTvel);

end

