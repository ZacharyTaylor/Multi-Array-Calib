
% samples = 10000;
% R = zeros(3,3,samples);
% out = zeros(3,samples);
% for i = 1:samples
%     a= [0.5 1 -0.5] + randn(1,3).*sqrt([0.01,0,0.09]);
%     b= [0.2 1.5 0.9] + randn(1,3).*sqrt([0,0.0,0]);
%     R(:,:,i) = V2R(a)*V2R(b);
%     out(:,i) = R2V(R(:,:,i));
% end
% 
% vR = var(R,[],3);
% vO = var(out,[],2);
% 
% a = V2R([0.2 1.5 0.9]);
% a*diag([0,0,0])*a'

samples = 1000;
out = zeros(3,samples);

r = [0 0 1];
v = [0.01,0,0.00];
b= [1 2 5]';
for i = 1:samples
    a= r + randn(1,3).*sqrt(v);
    
    out(:,i) = V2R(a)*b;
end

vO = var(out,[],2);

b = b.^2;
t = abs(CrossMat(v))*b;
t = diag(V2R(r)*diag(t)*V2R(r)');
vO = [vO,t];