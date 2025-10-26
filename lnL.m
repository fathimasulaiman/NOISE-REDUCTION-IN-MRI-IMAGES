function lnLA=lnL(mi,A,L,sigma2)

 n=numel(mi);% no of observations

 eq1=n*log(A/sigma2);
 eq2=L*(sum(mi/sigma2));
 eq3=zeros(3);
 eq4=zeros(3);
for ii=1:n
 eq3=eq3+((mi(ii).^2+A.^2)/(2*sigma2));   
 eq4=eq4+log(mi(ii)*A/sigma2);
end
lnLA=abs(eq1+eq2-eq3+eq4);