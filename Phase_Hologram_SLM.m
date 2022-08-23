function phase = Phase_Hologram_SLM(xyzp,weight,calibration)
% Create a phase hologram for sending to the SLM
%
%       phase = Phase_Hologram_SLM(xyzp,weight,calibration)
%
% Modified by Jesus Perez-Ortega, Aug 2019

SLMm = calibration.xSLM;
SLMn = calibration.ySLM;
objectiveRI = calibration.ObjectiveRI;
wavelength = calibration.Wavelenght;
objectiveNA = calibration.FocusModel(xyzp(:,3));


[u,v] = meshgrid(linspace(-SLMm/SLMm,SLMm/SLMm,SLMm),linspace(-SLMn/SLMn,SLMn/SLMn,SLMn));
nPoints = size(xyzp,1);
defocus = zeros(SLMm,SLMn,nPoints);

% Get the defocus of each point
for i = 1:nPoints
    defocus(:,:,i) = Defocus_Phase_SLM(SLMm,SLMn,objectiveNA(i),objectiveRI,wavelength);
end

% Create a phase hologram
plane = 0;
for i = 1:nPoints
    plane = plane+exp(1i.*(2*pi.*xyzp(i,1).*u+2*pi.*xyzp(i,2).*v...
               +xyzp(i,3).*defocus(:,:,i)))*weight(i);
end
phase = angle(plane)+pi;
phase = mod(phase,2*pi);
phase = phase.*255./(2*pi);