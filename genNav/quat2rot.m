function Q = quat2rot(q)
%QUAT2ROT quaternion to rotation matrix.
%   Q = QUAT2ROT(q) takes a 4-vector unit quaternion reprsented by q, 
%   (i.e. q = [q0;qx;qy;qz]) and returns the corresponding [3 x 3] 
%   orthonormal rotation matrix Q.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-10-2003      rme         Created.
%    09-20-2003      rme         Changed to store the scalar part as the
%                                first element instead of as the last element.

q0 = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

Q = zeros(3);
Q(1,1) = q0^2 + qx^2 - qy^2 - qz^2;
Q(1,2) = 2*(qx*qy - q0*qz);
Q(1,3) = 2*(qx*qz + q0*qy);
Q(2,1) = 2*(qy*qx + q0*qz);
Q(2,2) = q0^2 - qx^2 + qy^2 - qz^2;
Q(2,3) = 2*(qy*qz - q0*qx);
Q(3,1) = 2*(qz*qx - q0*qy);
Q(3,2) = 2*(qz*qy + q0*qx);
Q(3,3) = q0^2 - qx^2 - qy^2 + qz^2;
