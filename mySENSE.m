function [Is] = mySENSE(Sk,SMap,rate)
%
%mySENSE: Reconstruction of subsampled data using SENSE
%
% INPUTS:
%       Sk: subsampled k-space (without zeros!!)
%       SMap: sensitivity map for each coil
%             Must be of actual size Mx x My x Ncoils  
%       rate: Acceleratio rate
%
% OUTPUT
%	Is: Reconstructed (complex) x-space
%
% Implementation of  
% 
%     K. Pruessmann, M. Weiger, P.Boesieger
%     Sensitivity Encoding for fast MRI Scans (SENSE)
%     Magnetic Resonance in Medicine, 42:952-962 (1999)
%
% EXAMPLE
%       [Ics gmap]= mySENSE(SN,MapW, 2);
% 
%
% Based on implementartion "sense.m"  by Jim Ji, Texas A&M University.
%
% PARALLEL MRI TOOLBOX
%
% Santiago Aja-Fernandez, LPI
% www.lpi.tel.uva.es/~santi
% Valladolid 23/06/2011
%-----------------------------------------------------

[Mx,My,Ncoils] = size(Sk);
Nx2 = size(SMap,1);


if ceil(Nx2/Mx) ~= rate; 
    error('Size of data does not agree acceleration rate.'); 
elseif rate~=2
   error('SENSE only working for acceleration 2x'); 
end;

%K-space to x-space (reduced)
IxS = zeros(Mx,My,Ncoils);
for l=1:Ncoils
      IxS(:,:,l)=k2x(Sk(:,:,l),1);
end


%REMOVE ALIASING-----------------
Is=zeros(Nx2,My);

for jj=1:Mx %Lines
for ii=1:My %Columns

   if((jj+Mx*(rate-1))>Nx2)
     D1=(jj+Mx*(rate-1))-Nx2;
   else
     D1=0;
   end
   mp=squeeze(SMap(jj:Mx:(jj-D1+Mx*(rate-1)),ii,:));
   mp=transpose(mp);
   mp2=mp'*mp;
   I_mat=squeeze(IxS(jj,ii,:));
   Is(jj:Mx:(jj-D1+Mx*(rate-1)),ii) = transpose((inv(mp2)*mp'*I_mat));
end 
end



