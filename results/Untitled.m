res = zeros(100,6);
var = zeros(100,6);
for i = 1:35
    res(i,:) = results{i}.final(2,:);
    var(i,:) = results{i}.finalVar(2,:);
end

sqrt(abs(var));