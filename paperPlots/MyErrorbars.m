function [] = MyErrorbars( res, sd, colour )
%MYERRORBARS Summary of this function goes here
%   Detailed explanation goes here


plot(res,[colour '.'],'MarkerSize',20);

for i = 1:length(res)
    line([i,i],[res(i)-sd(i),res(i)+sd(i)],'color',colour);
end

