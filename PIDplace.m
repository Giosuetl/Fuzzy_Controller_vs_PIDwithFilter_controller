%Define the input and output variables for the MATLAB function, where a1, a0, b1,
%and b0 are the model parameters and Acl is the desired closed-loop characteristic
%polynomial. Enter the following program into the file
function [Kc,tauI,tauD,tauf]=PIDplace(a1,a0,b1,b0,Acl);
% Find the closed-loop performance parameters. Continue entering the following program into the file:
ac_3=Acl(2);
ac_2=Acl(3);
ac_1=Acl(4); 
ac_0=Acl(5);
%Form the following matrix and vector for the solution of the PID controller parameters,
%and solve the linear equation. Continue entering the following program into the file:
S_matrix=[1 b1 0 0; a1 b0 b1 0; a0 0 b0 b1; 0 0 0 b0];
Vec=[ac_3-a1;ac_2-a0;ac_1;ac_0];
contr_p=inv(S_matrix)*Vec;
%Convert the parameters into a PID controller with derivative filter. Continue entering
%the following program into the file:
L0=contr_p(1);
c2=contr_p(2);
c1=contr_p(3);
c0=contr_p(4);
tauf=1/L0;
tauI=c1/c0-tauf;
Kc=tauI*tauf*c0;
tauD=(c2*tauI*tauf-Kc*tauI*tauf)/(Kc*tauI);