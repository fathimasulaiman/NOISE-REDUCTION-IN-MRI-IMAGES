function [Leff,s2_eff]=effec_params_NCCHI(Si,Sigma)

% EFFEC_PARAMS_NCCHI
% 
% Calculate effective parameters for correlate multiple coil
%
% INPUT:
%         Si: Original signal in each coil (A_i)
%         Sigma= covariance matrix
%
% OUTPUT:
%	  LEff: Effective Number of coils
%         s2_eff: Effectiva variance of noise
%
%  Parameters for GRAPPA analysis are taken from
%
%     S. Aja-Fernández, A. Tristán-Vega, Influence of Noise Correlation in 
%     Multiple-Coil Statistical Models with Sum of Squares Reconstruction. 
%     Magnetic Resonance in Medicine. Volume 67, Issue 2, pages 580–585, 2012
% 
%
% PARALLEL MRI TOOLBOX
%
% Santiago Aja-Fernandez, LPI
% www.lpi.tel.uva.es/~santi
% Valladolid, 28/05/2012


AT2=sos(Si).^2;
[Mx My coils]=size(Si);
trS=trace(Sigma);
FbS2=sum((abs(Sigma(:))).^2);
for ii=1:Mx
for jj=1:My
Ai=squeeze(Si(ii,jj,:));
ACXA(ii,jj)=(Ai')*Sigma*(Ai);
end
end
Leff=real((AT2.*trS+(trS).^2)./(ACXA+FbS2));
s2_eff=real(trS./Leff);
