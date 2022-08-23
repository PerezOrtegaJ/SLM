function slmDetected = Open_SLM(trueFrames)
% Opens all the Boulder Nonlinear Systems SLM driver boards in the system.
% Assumes the devices are nematic (phase) SLMs.  Loads the library
% "InterfaceSLM.dll" into the MATLAB workspace.
%
%       slmDetected = Open_SLM(trueFrames)
%
% Modified by Jesus Per3ez-Ortega, Aug 2019

% Load library (original name was 'Interface.dll', changed to avoid conflict)
% It needs the following libraries:256PCIeBoard.dll, 512PCIeBoard.dll,
% BNSPCIeBoard.dll, Interface_thunk_pcwin64.dll, InterfaceSLM.dll and wdapi1021.dll
addpath('SLM libraries')
loadlibrary('InterfaceSLM.dll', @BNSPCIeInterface,'alias','interfaceSLM');

% Detect Try to connect
trueFrames = int32(trueFrames); 
slmDetected = calllib('interfaceSLM','Constructor',0,trueFrames,'PCIe512');

if ~slmDetected
    error('No SLM detected!')
end