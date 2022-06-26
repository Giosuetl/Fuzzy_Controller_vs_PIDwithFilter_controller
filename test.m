b1=0;
b0=0.5/0.002;
a1=0.05/0.002;
a0=0.629/0.002;
Ac1=[1 2*0.707*20 400];
Ac2=[1 40 400];
Acl=conv(Ac1,Ac2);
[Kc,tauI,tauD,tauf]=PIDplace(a1,a0,b1,b0,Acl)