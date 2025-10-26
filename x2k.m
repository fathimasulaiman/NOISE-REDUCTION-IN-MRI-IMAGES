function [Im]=x2k(ik)

% x2k
%
% Data in x-space to k-space
%
% INPUTS:
%	ik:     x-space data
%
% Note: Data format coresponds to method 1 in k2x
%
% PARALLEL MRI TOOLBOX
%
% Santiago Aja-Fernandez, LPI
% www.lpi.tel.uva.es/~santi
% Valladolid, 28/05/2012
Im=fftshift(fftshift(fft2(ik),1),2);


