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
Ts=Tp;      % sampling period
Tsim=min([Tp Ts])/100;  % simulation step-size
Tstep=0.050;
TSS=Tstep+0.050;
Tf=0.250;
%%% PMSM Parameters %%%%%%%%%%%%%
Pmrated=80e3;  % rated power
Tmrated=240;  % rated torque
Ismax=180;  % maximum phase current=111 Arms
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
%%% Battery + LC filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
VDC= 800;
rb = 80e-3;
Cb =500e-6;
Lb = 20e-6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Voltage-Source Inverter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vmax=VDC/sqrt(3);
Vp=VDC/2;
GVSI=1;
VDC0=0.8*VDC;
Qbat=72*3600;
load VSILoss
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Current Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tr=1e-3;  % desired response time
Psif0=1.0*Psif;
Rs0=1.0*Rs;
Ld0=1.0*Ld;
Lq0=1.0*Lq;
tid=Ld0/Rs0;
Kpd=3*Ld0/tr;
tiq=Lq0/Rs0;
Kpq=3*Lq0/tr;
Kid=Kpd/tid;
Kiq=Kpq/tiq;
s=tf('s');
Ci=Kpd+Kid/s;
Gi=1/(Ld*s+Rs);
Hi=Ci*Gi;
figure,margin(Hi),grid
RateTm=10000;  % Max rate of Tm = 10000 Nm/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MTPA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tmax=2*Tmrated;
Tref=-Tmax:Tmax/100:Tmax;
NT=length(Tref);
Idopt=zeros(1,NT);
Iqopt=zeros(1,NT);
for jj=1:NT
    Tm0=Tref(jj);
    Id0=0;
    kk=0;
    err=1;
    if Ld0<Lq0
        while err>1e-3 && kk<1e3
            Id1=Id0;
            Iq0=2/3*Tm0/(Pp*(Psif0+(Ld0-Lq0)*Id0));
            Id0=Psif0/(Lq0-Ld0)/2-sqrt(Psif0^2/(Lq0-Ld0)^2/4+Iq0^2);
            err=abs(Id1-Id0);
            kk=kk+1;
        end
    elseif Ld>Lq
        while err>1e-3 && kk<1e3
            Id1=Id0;
            Iq0=2/3*Tm0/(Pp*(Psif0+(Ld0-Lq0)*Id0));
            Id0=Psif0/(Lq0-Ld0)/2+sqrt(Psif0^2/(Lq0-Ld0)^2/4+Iq0^2);
            err=abs(Id1-Id0);
            kk=kk+1;
        end
    else
        Iq0=2/3*Tm0/(Pp*Psif0);
    end
    Idopt(jj)=Id0;
    Iqopt(jj)=Iq0;
end
figure(100),plot(Tref,Idopt,Tref,Iqopt,'linewidth',3),grid
legend('i_{dopt}','i_{qopt}')
xlabel('T_{m {ref}} [Nm]')
ylabel('I_{dq {opt}} [A]')
Idm=-Ismax:0.1:1;
if Ld0>Lq0, Idm=-Idm; end    
Iqmax=sqrt(Ismax^2-Idm.^2);
Idmax=[Idm,flip(Idm)];Iqmax=[Iqmax,-flip(Iqmax)];
Idm=-Ismax*1.2:1;
if Ld0>Lq0, Idm=-Idm; end    
Iqm=abs(Idm);
[IDM,IQM]=meshgrid(Idm,Iqm);
PSID=Psif0+Ld0*IDM;
PSIQ=Lq0*IQM;
TTM=3/2*Pp*(PSID.*IQM-PSIQ.*IDM);
WMM=Vmax./sqrt(PSID.^2+PSIQ.^2)/Pp;  % maximum mechanical speed
WMM=min(WMM,20000*pi/30);
figure(101),surf(IDM,IQM,TTM)
xlabel('i_d [A]')
ylabel('i_q [A]')
zlabel('T_m [Nm]')
figure(102),contour(IDM,IQM,TTM,'showtext','on','linewidth',2.5)
hold on
plot(Idopt,Iqopt,'b','linewidth',4),grid
plot(Idmax,Iqmax,'r','linewidth',5)
contour(IDM,IQM,WMM*30/pi,'showtext','on','linewidth',1)
hold off
xlabel('i_d [A]')
ylabel('i_q [A]')
legend('T_m [Nm]','MTPA','I_{max}','{\Omega}_{max} [rpm]')
if Ld0>Lq0, axis([0  Ismax*1.1 0 Ismax*1.1]),
else        axis([-Ismax*1.1 0 0 Ismax*1.1]), end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initial Values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Dyno=1;   % if 1, then fixed speed (dyno in speed mode)
Thetae0=0;
Wm0=2400*pi/30;
if Dyno==1
    Tm0=Tmrated*0.80;
else
    Tm0=(kL+f)*Wm0;
end
Pm0=Tm0*Wm0;
dTm0=0.20*Tmrated; 
Id0=0;
Iq0=Tm0/(3/2*Pp*Psif0);
MTPA=0;  % if 1, then MTPA strategy ON
if MTPA==1
    Id0=interp1(Tref,Idopt,Tm0);  % MTPA LUT 
    Iq0=interp1(Tref,Iqopt,Tm0);  % MTPA LUT
end
Ud0=Rs*Id0;
Uq0=Rs*Iq0;
Psid0=interpn(Idv,Iqv,Psidd,Id0,Iq0);
Psiq0=interpn(Idv,Iqv,Psiqq,Id0,Iq0);
Vd0=Ud0-Pp*Psiq0*Wm0;
Vq0=Uq0+Pp*Psid0*Wm0;
Vdq0=sqrt(Vd0^2+Vq0^2);
Pe0=3/2*(Vd0*Id0+Vq0*Iq0);
Idc0=Pe0/VDC;
SOC0=0.50;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Parameters loaded!')
