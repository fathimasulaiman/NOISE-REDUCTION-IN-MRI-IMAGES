function [Mn, params, M0, Kn, K0]=phantom_parallel(I,coils,Sigma,rho,parallel,rate)

% PHANTOM_PARALLEL: Simulates a multiple coil adquisition of a synthetic MR image
%
% INPUTS:
%	    I:        Input image. If I=0, the default image is used
%		      (A noise free MR slice from BrainWeb)
%		      Recomended Size 256x256	
%	    coils:    Number of acquisition coils
%           Sigma:    Coils covariance matrix
%	              if size(Sigma)=1x1  Sigma=variance of noise
%                     if size(Sigma)=(coils)x(coils) Sigma=covariance matrix
%           rho:      correlation coefficient between coils [0-1]
%                     If size(Sigma)>1 rho it is not used.
%                     otherwise:
%                           Covariance Matrix=Sigma*(eye(coils) +rho*(1-eye(coils))
%           parallel: Adquisición type:
%                       0: Multiple coils, no parallel acceleration
%                       1: SENSE (ONLY 2x acceleration works properly)
%                       2: GRAPPA
%           rate:      Subsampling rate (only for SENSE and GRAPPA) 
%		       Only regular subsampling is implemented. 
%                      For GRAPPA: number of acs lines can be included (Even number)
%                      Example:   rate=[2,32]
%                                 2x Acceleration
%                                 32 acs lines (EVEN number)
%		       For SENSE: number of first sampled line can be included 
%                                 (Smaller or equal than rate) rate(2)<=rate(1)
%                                 NOTE: Recomended 2x acceleration
%                      Example:   rate=[2,1]
%                                 2x Acceleration
%                                 Sampling starts in line number 1
%                      If only rate is included default first line is 1
%                                       
%
% OUTPUT
%             Mn:       Composite magnitude image (noisy)
%             M0:       Composite Magnitude image (non noisy, non accelerated)
%             Kn:       Original k space (noisy)
%             K0:       Original k space (non noisy, non accelerated)
%             params:   Other parameters
%                       params.Sigma:  covariance matrix
%			 -GRAPPA: 
%                              params.cf      GRAPPA reconstructions coefficients
%			       params.Wx      GRAPPA coefficientes (Map in x-space)
%                              params.Leff    Effective number of coils
%                              params.s2_eff  Effective variance of noise
%			       params.Ai      Reconstructed data in complex x-space
%                                             Same coefs than noisy case
%                                             but signal without noise.
%                        -SENSE: 
%                              (gfactor non implemented in this version)
%                               params.Ai     Reconstructed data in complex x-space
%                                             signal without noise. 
%
%                        -Multiple coil with correlation  
%                              params.Leff    Effective number of coils
%                              params.s2_eff  Effective variance of noise
%
%
%  Parameters for GRAPPA and nc-chi analysis are taken from
%
%     S. Aja-Fernández, A. Tristán-Vega, S. Hoge. Statistical Noise Analysis in 
%     GRAPPA using a parametrized non-central chi approximation model. 
%     Magnetic Resonance in Medicine. Volume 65, Issue 4, pages 1195–1206, 2011
%
%     S. Aja-Fernández, A. Tristán-Vega, Influence of Noise Correlation in 
%     Multiple-Coil Statistical Models with Sum of Squares Reconstruction. 
%     Magnetic Resonance in Medicine. Volume 67, Issue 2, pages 580–585, 2012
%
%
% 
%  ExampleS:
%
%      Mn=phantom_parallel(0,8,100,0.1,0,0); CORRELATED MULTIPLE COIL (NO ACCELERATION)
%      Mn=phantom_parallel(0,8,100,0.1,1,2); 8-COILS, GRAPPA, 2X ACCELERATION, 32 ACS
%      Mn=phantom_parallel(0,8,100,0.1,2,2); 8-COILS, SENSE, 2X ACCELERATION
%
% Santiago Aja-Fernandez, LPI
% www.lpi.tel.uva.es/~santi
% Valladolid, 
% Original: 23/06/2011, V2.0: 28/05/2012
wb=waitbar(0,'please wait........');
addpath common

if length(I)==1
	I=double(imread('mri.png'));
	I=I(2:end-1,2:end-1); %We force size 256x256
end

%1-- Simulation of coil sensitivity ----

[Mx,My]=size(I);	
MapW=sensitivity_map([Mx,My],coils); %Sensitivity Map
It=repmat(I,[1,1,coils]).*MapW;

waitbar(.2);

%2 Gaussian Noise in each coil---------------------------------
corr=0;
if (length(Sigma)>1)||(rho>0)
   corr=1;
end

if corr==0
	%No correlation between coils
	sigma=sqrt(Sigma); %Sigma: varianza of noise (sigma^2)
	Int=It+sigma.*(randn(size(It))+j.*randn(size(It)));
        params.Sigma=Sigma.*eye(coils);
else
	%correlation between coils
        if length(Sigma)==1
	  s1=Sigma; %s1=sigma^2
	  Sigma=s1.*(eye(coils)+rho.*(ones(coils)-eye(coils)));  % Matriz de covarianza
        end
	Nc=randn([Mx,My,coils])+j.*randn([Mx,My,coils]);	
   	[V,D] = eig(Sigma);
   	W = V*sqrt(D);
   	for ii=1:Mx
   	for jj=1:My
		Nc2(ii,jj,:)=W*(squeeze(Nc(ii,jj,:)));
   	end
   	end
        params.Sigma=Sigma;
        [params.Leff,params.s2_eff]=effec_params_NCCHI(It,params.Sigma);
	Int=It+Nc2;
end

ML=sos(Int); 	%Noisy Composite Magnitude Signal
ML0=sos(It); 	%Noise-free Composite Magnitude Signal
waitbar(.4);
%3- K Space------------------------------------------

SN=x2k(Int); %Fully sampled noisy
S0=x2k(It);  %Fully sampled without noise


if parallel==0
	Mn=ML; %Non Parallel output
	
elseif parallel==2

%4- GRAPPA------------------------------
% 4.1 k-space subsampling -----------------------------

waitbar(.5);
R=rate(1);
if numel(rate)==2
  Nacs=rate(2);
   if rem(Nacs,2)==1
     error('Number of ACS lines must be even');
   end 
else
  Nacs=32;
end

Z = zeros(Mx,1);
Z( 2:rate:end) = 1;  
ACS=(Mx/2) + (-(Mx/(Nacs/2)-2):(Mx/(Nacs/2)));	
Z(ACS) = 1;	%ACS lines 
Samp = find(Z);
k_sG=zeros(size(SN));
k_sA=zeros(size(S0));
k_sG(Samp,:,:)=SN(Samp,:,:); %Noisy
k_sA(Samp,:,:)=S0(Samp,:,:); %Non noisy

%4.2 GRAPPA Reconstruction
waitbar(.6);
[k_rG,Ix_G,G_coefs]=myGRAPPA([Mx My coils],k_sG,Samp,ACS,0);
[k_rA,Ix_A]=myGRAPPA([Mx My coils],k_sA,Samp,ACS,G_coefs);
Mn=sos(Ix_G); 
params.cf=G_coefs;
params.Ai=Ix_A;

%4.3 GRAPPA Noise analysis

params.Wx=coef_x(G_coefs,2,256,1,1)*256*256;
[params.Leff,params.s2_eff]=effec_params(Ix_A,params.Wx,params.Sigma);

elseif parallel==1

%5 SENSE---------------------------------------------

TasaM=rate(1);
if numel(rate)==2
  FsLn=rate(2);
   if rate(2)>rate(1)
     error('Initial sample line must be smaller or equal to acceleration rate');
   end 
else
  FsLn=1;
end
Ics= mySENSE(SN(FsLn:TasaM:end,:,:),MapW, TasaM);
params.Ai= mySENSE(S0(FsLn:TasaM:end,:,:),MapW, TasaM);
Mn=abs((Ics));   %Composite Magnitude Signal

else
   error('Parallel choice out of range. See help.')
end %END PARALLEL ACCELERATION

Kn=SN;
K0=S0;
M0=ML0;

waitbar(1);
close(wb);


