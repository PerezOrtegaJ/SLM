function weight = Correct_XYZ_Weights(xyzp,calibration)
% Compute the weights from calibration data
%
%       weight = Correct_XYZ_Weights(xyzp,calibration)
%
% Modified by Jesus Perez-Ortega, Aug 2019

WeightXY = calibration.WeightXY;
WeightZ = calibration.WeightZ;

weightXY = interp2(WeightXY.X,WeightXY.Y,WeightXY.Mat,xyzp(:,1),xyzp(:,2));
weight = abs(weightXY);

weightZ = interp1(WeightZ.Z,WeightZ.Mat,xyzp(:,3));
weight = weight.*abs(weightZ);