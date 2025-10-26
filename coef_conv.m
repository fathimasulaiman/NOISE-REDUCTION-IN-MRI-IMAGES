function wk=coef_conv(coef,tasa)

% Convolution matrix for GRAPPA in k-domain
% 
%  INPUTS:
% 		coef: grappa_coefs
% 		tasa: acceleration rate
%%
%
% PARALLEL MRI TOOLBOX
%
% Santiago Aja-Fernandez, LPI
% www.lpi.tel.uva.es/~santi
% Valladolid, 28/05/2012
%------------------------------------------
if length(size(coef))==2
	coef=ajusta_coef(coef);
end


[X,Y,Z,W]=size(coef);

if Z~=W
	error('Wrong nunber of coeffs');
end


wk=zeros([1+X*(tasa-1),Y,Z,Z]);

for kk=1:Z

%Matriz de convolución
M_conv=zeros([1+X*(tasa-1),Y,Z]);

for jj=1:Z
for gg=1:(tasa-1)
M_conv(gg,:,jj)=coef(1,:,jj,kk);
M_conv((end-(gg-1)),:,jj)=coef(2,:,jj,kk);
end

M_conv(tasa,2,kk)=1;

wk(:,:,jj,kk)=M_conv(:,:,jj);
end
end



