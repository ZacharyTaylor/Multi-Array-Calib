a = sData{1}.T_Skm1_Sk(41,:);
b = sData{2}.T_Skm1_Sk(41,:);

rA = V2R(a(4:6));
tA = a(1:3);
rB = V2R(b(4:6));
tB = b(1:3);

R = V2R(rotVec(2,:));
T = tranVec(2,:);

rB*R = R*rA
rB*T + tB = R*tA + T

(rB-eye)*T = R*tA - tB;
T = inv(rB-eye)*(R*tA-tB);

estA = R*tA;
estB = -tB;
estAB = rB-eye

estAB*T = -estA - estB

estAB*T + estA + estB = 0

