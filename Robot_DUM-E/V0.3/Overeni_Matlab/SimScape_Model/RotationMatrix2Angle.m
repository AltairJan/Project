function [alpha,beta,gamma] = RotationMatrix2Angle(R)
% Function transform rotation matrix into euler angles
% R - rotation matrix 3x3 

if size(R,1) ~= 3 || size(R,2) ~= 3
    error('Input matrix R must be 3x3.');
end
R = double(R);
R = round(R * 10^9)/10^9;
% R = round(R,9);

Sb = R(1,3);
Cb = sqrt(1-Sb^2);

Sa = -R(2,3)/Cb;
Ca = R(3,3)/Cb;

Sg = -R(1,2)/Cb;
Cg = R(1,1)/Cb;

% Sb = -R(3,1);
% Cb = sqrt(1-Sb^2);
% 
% Sa = R(3,2)/Cb;
% Ca = R(3,3)/Cb;
% 
% Sg = R(2,3)/Cb;
% Cg = R(1,3)/Cb;

alpha = atan2(Sa, Ca);
beta = atan2(Sb, Cb);
gamma = atan2(Sg, Cg);
end

