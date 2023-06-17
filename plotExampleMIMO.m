clc
close all
clear all



NumofRelative=0;
TimeofRelative=[];

NumofRelative1=0;
TimeofRelative1=[];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ŒÛ≤Ó¥Û
% bigx=load('wucahda_x1.mat');
% bigv=load('wucahda_v.mat');
%  bigu1=load('wucahda_faultu1.mat');
% bigu2=load('wucahda_faultu2.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%ŒÛ≤Ó–°
% smallx=load('wucahxiao_x1.mat');
smallv=load('u1.mat');
% smallu1=load('wucahxiao_faultu1.mat');
smalla=load('u2.mat');
%%%%%%%%%%

t=load('t.mat');t1=load('t.mat');
n1=size(t.t);
n2=size(t1.t);

n1=size(smallv.u1(:,1));
n2=size(smalla.u2(:,1));

for i=1:n1-1
    if smallv.u1(i)~= smallv.u1(i+1)
        TimeofRelative=[TimeofRelative,t.t(i+1)];
        NumofRelative=NumofRelative+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%5
% figure(5)
%  plot(TimeofRelative,NumofRelative, 'b');

figure(4)
subplot(2,1,1)
t_interval=zeros(NumofRelative-1,1);

  t_interval(1)= TimeofRelative(1);

for i=2:NumofRelative-1
        t_interval(i)=TimeofRelative(i)-TimeofRelative(i-1);
end


stem(TimeofRelative(1:NumofRelative-1),t_interval,'r')
xlabel('(a) Time(sec)','Fontsize', 11)
latexStr1 = ['The interevent times $t_{1,k+1}-t_{1,k}$'];
leg2=ylabel(latexStr1);
 set(leg2, 'interpreter', 'latex', 'Fontsize', 10)
 %title('The interevent times','Fontsize', 11)
 hold on
 
 
 
 
 
 
 for i=1:n2-1
    if smalla.u2(i)~= smalla.u2(i+1)
        TimeofRelative1=[TimeofRelative1,t1.t(i+1)];
        NumofRelative1=NumofRelative1+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%5
% figure(5)
%  plot(TimeofRelative,NumofRelative, 'b');

%figure(5)
subplot(2,1,2)
t1_interval1=zeros(NumofRelative1-1,1);

  t1_interval1(1)= TimeofRelative1(1);

for i=2:NumofRelative1-1
        t1_interval1(i)=TimeofRelative1(i)-TimeofRelative1(i-1);
end


stem(TimeofRelative1(1:NumofRelative1-1),t1_interval1,'r')
xlabel('(b) Time(sec)','Fontsize', 11)
latexStr2 = ['The interevent times $t_{2,k+1}-t_{2,k}$'];
leg3=ylabel(latexStr2);
 set(leg3, 'interpreter', 'latex', 'Fontsize', 10)
% title('The interevent times','Fontsize', 11)