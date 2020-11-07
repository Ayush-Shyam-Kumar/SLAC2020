clc; 
clear all;
%Parameters
%simulation range=8
%For starters try [3 1 3 6]
A = input('Enter length of crank: '); %crank
B = input('Enter length of coupler(shortest link): '); %coupler
C = input('Enter length of follower: '); %follower
D = input('Enter length of fixed link(longest link): '); %fixed link
R1=D/A;
R2=C/A;
flag1=1;
flag2=0;
if D>=A+B+C
    disp('Mechanism does not exist')
    flag1=0;
end
if ((B+D)-(A+C))>3
    disp('Mechanism goes imaginary')
    flag1=0;
end
if R1+R2<2
    disp('Watts Linkage criterion is not feasible')
    flag1=0;
end
if B+D==A+C
    disp('It is a change point mechansim and not watts linkage')
    flag1=0;
end
if flag1==1 && B+D>A+C
    disp('It follows triple rocker mechanism')
    flag2=1;
end
if flag2==1
t = 0:0.05:10; %time you need to simulate for
%code for sampling theta
theta=zeros(1,length(t));
m=acos((D^2+A^2-(C+B)^2)/(2*D*A)); %limit theta for which linkage wont become imaginary
a=linspace(0,m,7);
b=linspace(0,-m,7); %negative cycle
%x = 1;
y = 0;
z = -6;
for i=0:13
    if(rem(i,2)==0)
        theta(z+7:y+7)=a;
        z = z+7;
        y = y+7;
        theta((z+7):(y+7)) = flip(a);
        z = z+7;
        y = y+7;
    elseif(rem(i,2)~=0)
        theta(z+7:y+7)=b;
        z = z+7;
        y = y+7;
        theta((z+7):(y+7)) = flip(b);
        z = z+7;
        y = y+7;
    %x = x+1;
    end
end
%initialising link joints wrt coordinate system
P1 = [0;0]; %coordinates of intersection of crank and fixed link
P2 = A*[cos(theta); sin(theta)]; %coordinates of intersection of crank and coupler
E = sqrt(A^2 + D^2 - 2*A*D*cos(theta)); %resultant between A & D(crank & fixed link)
alfa = asin(A*sin(theta)./E); %angle between E and A
beta = acos((E.^2 + C^2 - B^2)./(2*E*C)); %angle between E and C(follower)
P3 = [D - C*cos(alfa+beta); C*sin(alfa+beta)]; %coordinates of intersection of follower and coupler
P4 = D*[1;0]; %coordinates of intersection of follower and fixed link
P5=zeros(2,201);
for i=1:201
    P5(1,i)=(P2(1,i)+P3(1,i))/2;
    P5(2,i)=(P2(2,i)+P3(2,i))/2;
end
P5_x = P5(1,:); %x coordinate of P5 vector
P5_y = P5(2,:); %y coordinate of P5 vector
P5_vx = diff(P5_x)./diff(t); %x coordinate of P5 velocity vector
P5_vy = diff(P5_y)./diff(t); %y coordinate of P5 velocity vector
P5_v = sqrt(P5_vx.^2 + P5_vy.^2); %magnitude of P5 velocity
P5_ax = diff(diff(P5_x)./diff(t)); %x coordinate of P5 acceleration vector
P5_ay =diff( diff(P5_y)./diff(t)); %y coordinate of P5 acceleration vector
P5_a = sqrt(P5_ax.^2 + P5_ay.^2); %magnitude of P5 acceleration
for i=1:length(t);
   ani = subplot(3,1,1);
   %viscircles draws circles with specified centers and radii onto the current axes
   P1_circle = viscircles(P1',0.05);
   P2_circle = viscircles(P2(:,i)',0.05);
   P3_circle = viscircles(P3(:,i)',0.05);
   P4_circle = viscircles(P4',0.05);
   P5_circle = viscircles(P5(:,i)',0.05,'color','b'); %the point on the coupler
   %plots a line in the current axes using the data in vectors x and y
   A_bar = line([P1(1) P2(1,i)],[P1(2) P2(2,i)]);
   B_bar = line([P2(1,i) P3(1,i)],[P2(2,i) P3(2,i)]);
   C_bar = line([P3(1,i) P4(1)],[P3(2,i) P4(2)]);   
   axis(ani,'equal');
   set(gca,'XLim',[0 8],'YLim',[-5 5]);
   str1 = 'P5';
   str2 = ['Time elapsed: '  num2str(t(i)) ' s'];
   P5_text = text(P5(1,i),P5(2,i)+0.4,str1);
   Time = text(-2,6,str2);
   pause(0.005);
   hold on;
   plot(P5_x,P5_y) %plots the path of P5
   if i<length(t)
    %stops simulation
    delete(P1_circle);
    delete(P2_circle);
    delete(P3_circle);
    delete(P4_circle);
    delete(P5_circle);
    delete(A_bar);
    delete(B_bar);
    delete(C_bar);
    delete(P5_text);
    delete(Time);
    %plots the velocity-time graph of P3
    vel = subplot(3,1,2);
    plot(vel,t(1:i),P5_v(1:i));
    set(vel,'XLim',[0 10],'YLim',[0 10]);
    xlabel(vel, 'Time (s)');
    ylabel(vel, 'Amplitude (m/s)');
    title(vel,'Velocity of P5');
    grid on;
    %plots the acceleration-time graph of P3
    acc = subplot(3,1,3);
    plot(acc,t(1:i-2),P5_a(1:i-2));
    set(acc,'XLim',[0 10],'YLim',[0 10]);
    xlabel(acc, 'Time (s)');
    ylabel(acc, 'Amplitude (m/s^2)');
    title(acc,'Acceleration of P5');
    grid on;
   end
end
P5_p=sqrt(P5_x.^2 + P5_y.^2); %resultant of position vectors
%plots for positions
figure
for i=0:length(P5_p)
    if i<length(t)
    pos=subplot(2,1,2);
    plot(pos,t(1:i),P5_p(1:i));
    set(pos,'XLim',[0 10],'YLim',[0 10]);
    xlabel(pos, 'Time (s)');
    ylabel(pos, 'Distance from origin');
    title(pos,'Distance of P5 from the orgin');
    end
end
pos2=subplot(2,1,1);
plot(P5_x,P5_y);
xlabel(pos2, 'x coordinate');
ylabel(pos2, 'y coordinate');
title(pos2,'Position plot of P5');
Qa=input('Enter the time for which you want calculate the vel/acc: ');
positionX=P5_x(20*Qa) %instantaneous x coordinate position
positionY=P5_y(20*Qa) %instantaneous y coordinate position
velocity=P5_v(20*Qa) %instantaneous velocity
acceleration=P5_a(20*Qa) %instantaneous acceleration
disp('the maximum velocity of P5 is :')
disp(max(P5_v))
disp('the minimum velocity of P5 is :')
disp(min(P5_v))
disp('the average velocity of P5 is :')
disp(mean(P5_a))
disp('the maximum acceleration of P5 is :')
disp(max(P5_a))
disp('the minimum acceleration of P5 is :')
disp(min(P5_a))
disp('the average acceleration of P5 is :')
disp(mean(P5_a))
end