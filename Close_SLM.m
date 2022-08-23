function Close_SLM()
%==========================================================================
%=   FUNCTION:  BNS_CloseSLM()
%=
%=   PURPOSE:   Closes the Boulder Nonlinear Systems SLM driver boards
%=              and unloads Interface.dll from the MATLAB Workspace
%==========================================================================
   calllib('interfaceSLM', 'SLMPower', 0);
   unloadlibrary('interfaceSLM');
end