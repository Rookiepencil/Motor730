clear
clc
close all

C32inv=2/3*[1    -1/2        -1/2;
            0  sqrt(3)/2  -sqrt(3)/2];
 
C32=[ 1       0;
    -1/2  sqrt(3)/2  
    -1/2 -sqrt(3)/2];

C33=C32*C32inv;

Tp=100e-6;  % switching period
Ts=Tp;     % sampling period
Tsim=min([Tp Ts])/100;  % simulation step-size
Tstep=0.20;
Tf=0.5;
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
figure(10)
surf(Idv,Iqv,Psidd')
xlabel('i_d [A]')
ylabel('i_q [A]')
zlabel('{\psi}_d [Wb]')
figure(11)
surf(Idv,Iqv,Psiqq')
xlabel('i_d [A]')
ylabel('i_q [A]')
zlabel('{\psi}_q [Wb]')
figure(12)
surf(Psidv,Psiqv,Idd')
xlabel('{\psi}_d [Wb]')
ylabel('{\psi}_q [Wb]')
zlabel('i_d [A]')
figure(13)
surf(Psidv,Psiqv,Iqq')
xlabel('{\psi}_d [Wb]')
ylabel('{\psi}_q [Wb]')
zlabel('i_q [A]')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Voltage-Source Inverter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
VDC=800;
Vmax=VDC/sqrt(3);
Vp=VDC/2;
GVSI=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Current Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tr=1e-3;  % desired response time
% Psif0=1.0*Psif;
% Rs0=1.0*Rs;
% Ld0=1.0*Ld;
% Lq0=1.0*Lq;
% tid=...;
% Kpd=...;
% tiq=...;
% Kpq=...;
% Kid=...;
% Kiq=...;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initial Values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Thetae0=0;
Wm0=1500*pi/30;
Tm0=(kL+f)*Wm0;
Pm0=Tm0*Wm0;
dTm0=0.20*Tmrated; % 10*fix(0.20*Tm0/10);
Id0=0;
Iq0=2/3*Tm0/(Pp*(Psif+(Ld-Lq)*Id0));
Ud0=Rs*Id0;
Uq0=Rs*Iq0;
Psid0=interpn(Idv,Iqv,Psidd,Id0,Iq0);
Psiq0=interpn(Idv,Iqv,Psiqq,Id0,Iq0);
Vd0=Ud0-Pp*Psiq0*Wm0;
Vq0=Uq0+Pp*Psid0*Wm0;
Vdq0=sqrt(Vd0^2+Vq0^2);
Pe0=3/2*(Vd0*Id0+Vq0*Iq0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Parameters loaded!')
