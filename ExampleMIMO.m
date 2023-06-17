clear all
 close all
 clc
 
 t_threshold=0.0001;% simulaiton stepsize
 T=30;% simulaiton time
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%下边为误差小参数
 g1=0;g2=0;
 lam11=0.5;lam12=0.5;lam21=0.5;lam22=0.5; 
d11=10;d12=10;d21=5;d22=5;

k11=10;k12=5;k21=6;k22=6;
% k11=15;k12=8;k21=10;k22=10;
% k11=20;k12=10;k21=15;k22=15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t=zeros(T/t_threshold,1);
x11=zeros(T/t_threshold,1);
x12=zeros(T/t_threshold,1);
x21=zeros(T/t_threshold,1);
x22=zeros(T/t_threshold,1);
law11=zeros(T/t_threshold,1);law12=zeros(T/t_threshold,1);law13=zeros(T/t_threshold,1);law14=zeros(T/t_threshold,1);law15=zeros(T/t_threshold,1);
law21=zeros(T/t_threshold,1);law22=zeros(T/t_threshold,1);law23=zeros(T/t_threshold,1);law24=zeros(T/t_threshold,1);law25=zeros(T/t_threshold,1);


cumulative=zeros(T/t_threshold,1);
cumulative2=zeros(T/t_threshold,1);
Trig=zeros(T/t_threshold,1);
% Trig1=zeros(T/t_threshold,1);
Trig2=zeros(T/t_threshold,1);
TrigX=zeros(T/t_threshold,1);

w1=zeros(T/t_threshold,1);%continuous control signal 
u1=zeros(T/t_threshold,1);%event triggered contorl signal
alph11=zeros(T/t_threshold,1);%continuous control signal 
baralph11=zeros(T/t_threshold,1);%event triggered contorl signal
w2=zeros(T/t_threshold,1);%continuous control signal 
u2=zeros(T/t_threshold,1);%event triggered contorl signal
alph21=zeros(T/t_threshold,1);%continuous control signal 
baralph21=zeros(T/t_threshold,1);%event triggered contorl signal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x11_0=0.2;x12_0=0.2;x21_0=0.1;x22_0=0.1;
law11_0=0;law12_0=0;law13_0=0;law14_0=0;law15_0=0;
law21_0=0;law22_0=0;law23_0=0;law24_0=0;law25_0=0;
law31_0=0;law32_0=0;law33_0=0;law34_0=0;law35_0=0;
law41_0=0;law42_0=0;law43_0=0;law44_0=0;law45_0=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x11(1)=x11_0;x12(1)=x12_0;x21(1)=x21_0;x22(1)=x22_0;
law11(1)=law11_0;law12(1)=law12_0;law13(1)=law13_0;law14(1)=law14_0;law15(1)=law15_0;
law21(1)=law21_0;law22(1)=law22_0;law23(1)=law23_0;law24(1)=law24_0;law25(1)=law25_0;
law31(1)=law31_0;law32(1)=law32_0;law33(1)=law33_0;law34(1)=law34_0;law35(1)=law35_0;
law41(1)=law41_0;law42(1)=law42_0;law43(1)=law43_0;law44(1)=law44_0;law45(1)=law45_0;

rt1=[];
rr1=[];
tk=0;
Trig=[];
   cumulative=[];
rt3=0;
rt2=[];
O=0;

% rt11=[];
% rr11=[];
% tk1=0;
% Trig1=[];
%    cumulative1=[];
% rt31=0;
% rt21=[];
% O1=0;

rt12=[];
rr12=[];
tk2=0;
Trig2=[];
   cumulative2=[];
rt32=0;
rt22=[];
O2=0;

% rt112=[];
% rr112=[];
% tk3=0;
% Trig3=[];
%    cumulative3=[];
% rt33=0;
% rt23=[];
% O3=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:1:T/ t_threshold-1
    t(i)=(i-1)*t_threshold;
     count=(i-1)*0.01;  % 决定图5的横坐标  
%基函数
S11(1)=exp(-(x11(i)+2)^2);
S11(2)=exp(-(x11(i)+1)^2);
S11(3)=exp(-(x11(i))^2);
S11(4)=exp(-(x11(i)-1)^2);
S11(5)=exp(-(x11(i)-2)^2);
S11=S11/sum(S11);
S12(1)=exp(-(x11(i)+2)^2)*exp(-(x12(i)+2)^2);
S12(2)=exp(-(x11(i)+1)^2)*exp(-(x12(i)+1)^2);
S12(3)=exp(-(x11(i))^2)*exp(-(x12(i))^2);
S12(4)=exp(-(x11(i)-1)^2)*exp(-(x12(i)-1)^2);
S12(5)=exp(-(x11(i)-2)^2)*exp(-(x12(i)-2)^2);
S12=S12/sum(S12);
%基函数
S21(1)=exp(-(x21(i)+2)^2);
S21(2)=exp(-(x21(i)+1)^2);
S21(3)=exp(-(x21(i))^2);
S21(4)=exp(-(x21(i)-1)^2);
S21(5)=exp(-(x21(i)-2)^2);
S21=S21/sum(S21);
S22(1)=exp(-(x21(i)+2)^2)*exp(-(x22(i)+2)^2)*exp(-(x11(i)+2)^2)*exp(-(x12(i)+2)^2);
S22(2)=exp(-(x21(i)+1)^2)*exp(-(x22(i)+1)^2)*exp(-(x11(i)+1)^2)*exp(-(x12(i)+1)^2);
S22(3)=exp(-(x21(i))^2)*exp(-(x22(i))^2)*exp(-(x11(i))^2)*exp(-(x12(i))^2);
S22(4)=exp(-(x21(i)-1)^2)*exp(-(x22(i)-1)^2)*exp(-(x11(i)-1)^2)*exp(-(x12(i)-1)^2);
S22(5)=exp(-(x21(i)-2)^2)*exp(-(x22(i)-2)^2)*exp(-(x11(i)-2)^2)*exp(-(x12(i)-2)^2);
S22=S22/sum(S22);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
law1a=[law11(i);law12(i);law13(i);law14(i);law15(i)];
law2b=[law21(i);law22(i);law23(i);law24(i);law25(i)];
law3c=[law31(i);law32(i);law33(i);law34(i);law35(i)];
law4d=[law41(i);law42(i);law43(i);law44(i);law45(i)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
y1d=sin(t(i));y1dd=cos(t(i));
y2d=sin(t(i));y2dd=cos(t(i));
z11=x11(i)-y1d;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r1=1;T1=8;q1=0.1;
% r1=1;T1=5;q1=0.05;
% r1=1;T1=2;q1=0.02;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DL1=(r1*(sin((pi/(2*T1))*(T1-t(i))))^3)*(t(i)>=0&t(i)<=T1)+0*(t(i)>T1);
D1=DL1+q1;
% dotD1=(-((3*r1*pi)/(2*T))*cos((pi/(2*T1))*(T1-t(i)))*(sin((pi/(2*T1))*(T1-t(i))))^2)*(t(i)>=0&t(i)<=T1)+0*(t(i)>T1);
% f1=-(2*D1*z11^3)/((D1*D1-z11*z11)^2);
% F1=dotD1*f1; 
omga1=(D1*D1*(D1*D1+z11*z11))/((D1*D1-z11*z11)^2);
elta1=(D1*D1*z11)/((D1-z11)*(D1+z11)); 

% f1=-(2*D1*z11)/((D1*D1-z11*z11)^2);
% F1=dotD1*f1; 
% omga1=(D1*D1+z11*z11)/((D1*D1-z11*z11)^2);
% elta1=(z11)/((D1-z11)*(D1+z11));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alph11(i)=-((k11+1)*elta1)/omga1-omga1*elta1-S11*law1a; %c1(i)=alph1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alph12=(-k12*(x12(i)-alph11(i))-S12*law2b)*2;
dlta12=0.5;umin12=0.6; h12=1;
dlta11=0.6;umin11=0.6; h11=1;
w1(i)=alph12;
    if i==1 || abs(w1(i)-u1(i-1))>=dlta12*tanh(abs(w1(i)/h12))+umin12
        u1(i)=w1(i);
        baralph11(i)=alph11(i);
          g1=g1+1;
          ti=i;
             rr1=[rr1;count-tk];
       tk=count;
       rt1=[rt1;count];
       
          
       rt3=rt3+1;
        rt2=[rt2;rt3];
       
%       rt2=[rt2;sum(rt2)];

cumulative=[cumulative,sum(rt3)];
       O=O+1;
   
       
       J=rt2(O);
       
       UU{J}=i;
       UUU{J}=J;
    else
        if i==1 || abs(alph11(i)-baralph11(i-1))>=dlta11*tanh(abs(alph11(i)/h11))+umin11
        baralph11(i)=alph11(i);
        u1(i)=w1(i);
          g1=g1+1;
          ti=i;
             rr1=[rr1;count-tk];
       tk=count;
       rt1=[rt1;count];
       
          
       rt3=rt3+1;
        rt2=[rt2;rt3];
        cumulative=[cumulative,sum(rt3)];
       O=O+1;
   
       
       J=rt2(O);
       
       UU{J}=i;
       UUU{J}=J;
        else
           baralph11(i)=baralph11(i-1);
           u1(i)=u1(i-1);
        end
    end  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
z21=x21(i)-y2d;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r2=1;T2=8;q2=0.1;
% r2=1;T2=5;q2=0.05;
% r2=1;T2=2;q2=0.02;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DL2=(r2*(sin((pi/(2*T2))*(T2-t(i))))^3)*(t(i)>=0&t(i)<=T2)+0*(t(i)>T2);
D2=DL2+q2;
%dotD2=(-((3*r2*pi)/(2*T2))*cos((pi/(2*T2))*(T2-t(i)))*(sin((pi/(2*T2))*(T2-t(i))))^2)*(t(i)>=0&t(i)<=T2)+0*(t(i)>T2);
% f2=-(2*D2*z21^3)/((D2*D2-z21*z21)^2);
% F2=dotD2*f2; 
omga2=(D2*D2*(D2*D2+z21*z21))/((D2*D2-z21*z21)^2);
elta2=(D2*D2*z21)/((D2-z21)*(D2+z21)); 

% f2=-(2*D2*z21)/((D2*D2-z21*z21)^2);
% F2=dotD2*f2; 
% omga2=(D2*D2+z21*z21)/((D2*D2-z21*z21)^2);
% elta2=(z21)/((D2-z21)*(D2+z21));

alph21(i)=-((k21+1)*elta2)/omga2-omga2*elta2-S21*law3c; %c1(i)=alph1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alph22=-k22*(x22(i)-alph21(i))-S22*law4d;
dlta22=0.5;umin22=0.1; h22=1;
dlta21=0.5;umin21=0.2; h21=1;
w2(i)=alph22;
    if i==1 || abs(w2(i)-u2(i-1))>=dlta22*tanh(abs(w2(i)/h22))+umin22
        u2(i)=w2(i);
        baralph21(i)=alph21(i);
          g2=g2+1;
          ti=i;
             rr12=[rr12;count-tk2];
       tk2=count;
       rt12=[rt12;count];
       
          
       rt32=rt32+1;
        rt22=[rt22;rt32];
       
%       rt2=[rt2;sum(rt2)];

cumulative2=[cumulative2,sum(rt32)];
       O2=O2+1;
   
       
       J2=rt22(O2);
       
       UU{J2}=i;
       UUU{J2}=J2;
    else
        if i==1 || abs(alph21(i)-baralph21(i-1))>=dlta21*tanh(abs(alph21(i)/h21))+umin21
        baralph21(i)=alph21(i);
        u2(i)=w2(i);
          g2=g2+1;
          ti=i;
             rr12=[rr12;count-tk2];
       tk2=count;
       rt12=[rt12;count];
       
          
       rt32=rt32+1;
        rt22=[rt22;rt32];
        cumulative2=[cumulative2,sum(rt32)];
       O2=O2+1;
   
       
       J2=rt22(O2);
       
       UU{J2}=i;
       UUU{J2}=J2;
        else
           baralph21(i)=baralph21(i-1);
           u2(i)=u2(i-1);
        end
    end      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
I1=[1 0 0 0 0];
I2=[0 1 0 0 0];
I3=[0 0 1 0 0];
I4=[0 0 0 1 0];
I5=[0 0 0 0 1];
%系统的动态方程
x11(i+1)=t_threshold*((2+cos(x11(i)))*x12(i)+0.5*x11(i)*x11(i))+x11(i);
x12(i+1)=t_threshold*(x11(i)*x12(i)+(1+0.5*sin(x11(i)*x12(i)))*u1(i))+x12(i);
% x21(i+1)=t_threshold*(-x21(i)*sin(x21(i))*sin(x21(i)))+x21(i);
% x22(i+1)=t_threshold*(x11(i)*x22(i)*sin(x12(i))+u2(i))+x22(i);
x21(i+1)=t_threshold*((2+sin(x21(i)))*x22(i)+0.3*sin(x21(i)*x21(i)))+x21(i);
x22(i+1)=t_threshold*((2+cos(x21(i)*x22(i)))*u2(i)+0.5*sin(x11(i)*x21(i)*x12(i)*x22(i)))+x22(i);

%自适应率
% law1=[law11(i);law12(i);law13(i);law14(i);law15(i)];
law11(i+1)=t_threshold*(-lam11*I1*law1a+d11*elta1*omga1*I1*S11')+law11(i);
law12(i+1)=t_threshold*(-lam11*I2*law1a+d11*elta1*omga1*I2*S11')+law12(i);
law13(i+1)=t_threshold*(-lam11*I3*law1a+d11*elta1*omga1*I3*S11')+law13(i);
law14(i+1)=t_threshold*(-lam11*I4*law1a+d11*elta1*omga1*I4*S11')+law14(i);
law15(i+1)=t_threshold*(-lam11*I5*law1a+d11*elta1*omga1*I5*S11')+law15(i);
% law2=[law21(i);law22(i);law23(i);law24(i);law25(i)];
law21(i+1)=t_threshold*(-lam12*I1*law2b+d12*(x12(i)-alph11(i))*I1*S12')+law21(i);
law22(i+1)=t_threshold*(-lam12*I2*law2b+d12*(x12(i)-alph11(i))*I2*S12')+law22(i);
law23(i+1)=t_threshold*(-lam12*I3*law2b+d12*(x12(i)-alph11(i))*I3*S12')+law23(i);
law24(i+1)=t_threshold*(-lam12*I4*law2b+d12*(x12(i)-alph11(i))*I4*S12')+law24(i);
law25(i+1)=t_threshold*(-lam12*I5*law2b+d12*(x12(i)-alph11(i))*I5*S12')+law25(i);
% law1=[law11(i);law12(i);law13(i);law14(i);law15(i)];
law31(i+1)=t_threshold*(-lam21*I1*law3c+d21*elta2*omga2*I1*S21')+law31(i);
law32(i+1)=t_threshold*(-lam21*I2*law3c+d21*elta2*omga2*I2*S21')+law32(i);
law33(i+1)=t_threshold*(-lam21*I3*law3c+d21*elta2*omga2*I3*S21')+law33(i);
law34(i+1)=t_threshold*(-lam21*I4*law3c+d21*elta2*omga2*I4*S21')+law34(i);
law35(i+1)=t_threshold*(-lam21*I5*law3c+d21*elta2*omga2*I5*S21')+law35(i);
% law2=[law21(i);law22(i);law23(i);law24(i);law25(i)];
law41(i+1)=t_threshold*(-lam22*I1*law4d+d22*(x22(i)-alph21(i))*I1*S22')+law41(i);
law42(i+1)=t_threshold*(-lam22*I2*law4d+d22*(x22(i)-alph21(i))*I2*S22')+law42(i);
law43(i+1)=t_threshold*(-lam22*I3*law4d+d22*(x22(i)-alph21(i))*I3*S22')+law43(i);
law44(i+1)=t_threshold*(-lam22*I4*law4d+d22*(x22(i)-alph21(i))*I4*S22')+law44(i);
law45(i+1)=t_threshold*(-lam22*I5*law4d+d22*(x22(i)-alph21(i))*I5*S22')+law45(i);
la1(i)=norm(law1a);
la2(i)=norm(law2b);
la3(i)=norm(law3c);
la4(i)=norm(law4d);
end


figure(1)% tracking error
subplot(2,2,1)
plot(t(1:T/t_threshold-1),x11(1:T/t_threshold-1),'r');
hold on
plot(t(1:T/t_threshold-1),sin(t(1:T/t_threshold-1)),'b');
xlabel('(a) Time(sec)')
ylabel('$y_1$ and $y_{1,d}$','Interpreter','latex')
legend('$y_1$','$y_{1,d}$')
subplot(2,2,2)
% figure(2)
plot(t(1:T/t_threshold-1),x11(1:T/t_threshold-1)-sin(t(1:T/t_threshold-1)),'b');
hold on
plot(t(1:T/t_threshold-1),-((r1*(sin((pi/(2*T1))*(T1-t(1:T/t_threshold-1)))).^3).*(t(1:T/t_threshold-1)>=0&t(1:T/t_threshold-1)<=T1)+0.*(t(1:T/t_threshold-1)>T1)+q1),'-.r',t(1:T/t_threshold-1),(r1*(sin((pi/(2*T1))*(T1-t(1:T/t_threshold-1)))).^3).*(t(1:T/t_threshold-1)>=0&t(1:T/t_threshold-1)<=T1)+0.*(t(1:T/t_threshold-1)>T1)+q1,'-.r');
xlabel('(b) Time(sec)')
ylabel('The tracking error $z_{1,1}$','Interpreter','latex')
legend('$z_{1,1}$','$- D_1(t)$','$D_1(t)$')
%figure(7)% tracking error
subplot(2,2,3)
plot(t(1:T/t_threshold-1),x21(1:T/t_threshold-1),'r');
hold on
plot(t(1:T/t_threshold-1),sin(t(1:T/t_threshold-1)),'b');
xlabel('(c) Time(sec)')
ylabel('$y_2$ and $y_{2,d}$','Interpreter','latex')
legend('$y_2$','$y_{2,d}$')
%figure(8)
subplot(2,2,4)
plot(t(1:T/t_threshold-1),x21(1:T/t_threshold-1)-sin(t(1:T/t_threshold-1)),'b');
hold on
plot(t(1:T/t_threshold-1),-((r2*(sin((pi/(2*T2))*(T2-t(1:T/t_threshold-1)))).^3).*(t(1:T/t_threshold-1)>=0&t(1:T/t_threshold-1)<=T2)+0.*(t(1:T/t_threshold-1)>T2)+q2),'-.r',t(1:T/t_threshold-1),(r2*(sin((pi/(2*T2))*(T2-t(1:T/t_threshold-1)))).^3).*(t(1:T/t_threshold-1)>=0&t(1:T/t_threshold-1)<=T2)+0.*(t(1:T/t_threshold-1)>T2)+q2,'-.r');
xlabel('(d) Time(sec)')
ylabel('The tracking error $z_{2,1}$','Interpreter','latex')
legend('$z_{2,1}$','$-D_2(t)$','$D_2(t)$')

% figure(3)
% subplot(2,1,1)
% plot(t(1:T/t_threshold-1),w1(1:T/t_threshold-1),'b',t(1:T/t_threshold-1),u1(1:T/t_threshold-1),'r');
% xlabel('(a) Time(sec)')
% ylabel('Control input $w_1$ and $u_1$','Interpreter','latex')
% legend('$w_1$','$u_1$')
% %figure(11)
% subplot(2,1,2)
% plot(t(1:T/t_threshold-1),w2(1:T/t_threshold-1),'b',t(1:T/t_threshold-1),u2(1:T/t_threshold-1),'r');
% xlabel('(b) Time(sec)')
% ylabel('Control input $w_2$ and $u_2$','Interpreter','latex')
% legend('$w_2$','$u_2$')

figure(3)
subplot(2,1,1)
plot(t(1:T/t_threshold-1),u1(1:T/t_threshold-1),'b');
xlabel('(a) Time(sec)')
ylabel('Control input $u_1$','Interpreter','latex')
legend('$u_1$')
%figure(11)
subplot(2,1,2)
plot(t(1:T/t_threshold-1),u2(1:T/t_threshold-1),'b');
xlabel('(b) Time(sec)')
ylabel('Control input $u_2$','Interpreter','latex')
legend('$u_2$')


figure(4)
subplot(2,1,1)
plot(t(1:T/t_threshold-1),la1(1:T/t_threshold-1),'b',t(1:T/t_threshold-1),la2(1:T/t_threshold-1),'r');
xlabel('(a) Time(sec)')
ylabel('Adaptive parameters $\|\hat{W}_{1,i}\|$','Interpreter','latex')
legend('$\|\hat{W}_{1,1}\|$','$\|\hat{W}_{1,2}\|$')
% figure(12)
subplot(2,1,2)
plot(t(1:T/t_threshold-1),la3(1:T/t_threshold-1),'b',t(1:T/t_threshold-1),la4(1:T/t_threshold-1),'r');
xlabel('(b) Time(sec)')
ylabel('Adaptive parameters $\|\hat{W}_{2,i}\|$','Interpreter','latex')
legend('$\|\hat{W}_{2,1}\|$','$\|\hat{W}_{2,2}\|$')


 figure(5)
 plot(rt1,cumulative.*9, 'r');
grid on;
xlabel('Time(esc)')
ylabel('Number of events of the first subsystem')

% figure(6)
% plot(t(1:T/t_threshold-1),alph11(1:T/t_threshold-1),'b',t(1:T/t_threshold-1),baralph11(1:T/t_threshold-1),'r');
% xlabel('Time(sec)')
% %ylabel('Control input')
% legend('$alph1$','$baralph1$')

 figure(13)
 plot(rt12,cumulative2.*9, 'r');
grid on;
xlabel('Time(esc)')
ylabel('Number of events of the 2th subsystem')

% figure(14)
% plot(t(1:T/t_threshold-1),alph21(1:T/t_threshold-1),'b',t(1:T/t_threshold-1),baralph21(1:T/t_threshold-1),'r');
% xlabel('Time(sec)')
% %ylabel('Control input')
% legend('$alph21$','$baralph21$')