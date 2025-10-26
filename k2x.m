function [Im]=k2x(ik,method)

% K2X
%
% Data in K-space to X-space
%
% INPUTS:
%	ik:     k-space data
%       method: Different data formart (default 1)
%
% PARALLEL MRI TOOLBOX
%
% Santiago Aja-Fernandez, LPI
% www.lpi.tel.uva.es/~santi
% Valladolid, 28/05/2012

if method==1
	Im=ifft2(fftshift(fftshift(ik,1),2));
elseif method==2
	Im=ifftshift(ifft2(ifftshift(ik)));
elseif method==3
	Im=fftshift(ifft2(ifftshift(ik)),2); 
end

