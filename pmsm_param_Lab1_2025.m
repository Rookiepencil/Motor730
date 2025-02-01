clear
clc
close all

C32inv=2/3*[1    -1/2        -1/2;
            0  sqrt(3)/2  -sqrt(3)/2];
 
C32=[ 1       0;
    -1/2  sqrt(3)/2  
    -1/2 -sqrt(3)/2];

C33=C32*C32inv;

Tsim=1e-6;  % simulation step-size
%%% PMSM Parameters %%%%%%%%%%%%%
Pmrated=80e3;  % rated power
Tmrated=240;   % rated torque
Ismax=180;     % phase peak current
Rs= 60e-3;
Ld=1.0e-3;
Lq=2.0e-3;
Psif=167e-3;
Pp=4;
kL=0.750;
f= 15e-3;
J=100e-3;
load LUTdq     % load id and iq LUTs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Voltage-Source Inverter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vm=300;  % sinusoidal voltage (model validation test)
fm=100/3;  % sinusoidal frequency (model validation test)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initial Values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Thetae0=0;
Wm0=0;
Tm0=(kL+f)*Wm0;
Id0=0;
Iq0=2/3*Tm0/(Pp*Psif);
Psid0=interpn(Idv,Iqv,Psidd,Id0,Iq0);
Psiq0=interpn(Idv,Iqv,Psiqq,Id0,Iq0);
Vd0=Rs*Id0-Pp*Psiq0*Wm0;
Vq0=Rs*Iq0+Pp*Psid0*Wm0;
Pm0=Tm0*Wm0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Parameters loaded!')
