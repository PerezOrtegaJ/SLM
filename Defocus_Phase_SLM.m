function defocus = Defocus_Phase_SLM(SLMm,SLMn,objectiveNA,objectiveRI,illuminationWavelength)
% Defocus based on objective NA
%
%       defocus = Defocus_Phase_SLM(SLMm,SLMn,objectiveNA,objectiveRI,illuminationWavelength)
%
% Modified by Jesus Perez-Ortega, Aug 2019

xlm = linspace(-1,1,SLMm);
xln = linspace(-1,1,SLMn);
[fX,fY] = meshgrid(xlm,xln);
[~,rho] = cart2pol(fX,fY);
alpha = asin((objectiveNA./objectiveRI));
k = 2*pi/illuminationWavelength;

% from 'Three dimensional imaging and photostimulation by remote focusing and holographic light patterning'
c2 = (objectiveRI*k*(sin(alpha)^2)/(8*pi*sqrt(3))).*(1+(1/4)*(sin(alpha)^2)+(9/80)*(sin(alpha)^4)+(1/16)*(sin(alpha)^6));
c4 = (objectiveRI*k*(sin(alpha)^4)/(96*pi*sqrt(5))).*(1+(3/4)*(sin(alpha)^2)+(15/18)*(sin(alpha)^4));
c6 = (objectiveRI*k*(sin(alpha)^6)/(640*pi*sqrt(7))).*(1+(5/4)*(sin(alpha)^2));
z2 = sqrt(3).*(2.*rho.^2-1);
z4 = sqrt(5).*(6.*rho.^4-6.*rho.^2+1);
z6 = sqrt(7).*(20.*rho.^6-30.*rho.^4+12.*rho.^2-1);

% Get the defocuse image
defocus = 2*pi.*(c2.*z2+c4.*z4+c6.*z6);