function [w2]=ajusta_coef(w)

% Adjust size of matrix of GRAPPA coefficients
% NORMALIZATION
% From 8x48 or 48x8 GRAPPA coefs
% Build [2x3x8x8] matrix
% 
% PARALLEL MRI TOOLBOX
%
% Santiago Aja-Fernandez, LPI
% www.lpi.tel.uva.es/~santi
% Valladolid, 28/05/2012

[M,N]=size(w);
if M>N
w=w';
end
[M,N]=size(w);
%M=8
%N=8x3x2

w2=zeros([2,3,M,M]);

for kk=1:M

  Tw=reshape(w(kk,:),[3*M,2]);

  w2(:,:,:,kk)=reshape(Tw',[2,3,M]);

end




