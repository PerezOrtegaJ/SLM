function Load_Image_SLM(idSLM,image,optimization)
% Loads a image into a memory frame on the SLM driver board.
% WARNING - Loading the same memory frame that is currently
% being viewed on the SLM can result in corrupted images.
%
%   INPUTS:     ImageMatrix - A 512x512 matrix or 256x256 of integers, each 
%                             within range 0..255, corresponding to the voltage
%                             to be applied to the SLM pixel.
%
% Modified by Jesus Perez-Ortega, Aug 2019

if nargin==3
    image = mod(image+optimization,256);
end

pImage = libpointer('uint8Ptr',image); 
calllib('interfaceSLM','WriteImage',idSLM,pImage,512);