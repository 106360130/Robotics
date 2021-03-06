%*************************%
% PUMA 560 Cartesian Move
%*************************%
%{
目前theta4 5 6有問題
%}

clear all
close all

function_path = "C:\Users\RTES\Desktop\Jakob\Robotics\Project\project_2\Reference\inverse_kinematics";

%**************************************************%
% (一) defind the orientation and position of A,B,C
%**************************************************%

A = [0 0 -1 40; -1 0 0 -30; 0 1 0 10; 0 0 0 1];
B = [1 0 0 30; 0 -1 0 30; 0 0 -1 20; 0 0 0 1];
C = [0 1 0 40; 0 0 -1 20; -1 0 0 -30; 0 0 0 1];


sampling_rate = 0.002;
r = 1;

nA=[A(1,1);A(2,1);A(3,1)];
oA=[A(1,2);A(2,2);A(3,2)];
aA=[A(1,3);A(2,3);A(3,3)];
pA=[A(1,4);A(2,4);A(3,4)];
nB=[B(1,1);B(2,1);B(3,1)];
oB=[B(1,2);B(2,2);B(3,2)];
aB=[B(1,3);B(2,3);B(3,3)];
pB=[B(1,4);B(2,4);B(3,4)];
nC=[C(1,1);C(2,1);C(3,1)];
oC=[C(1,2);C(2,2);C(3,2)];
aC=[C(1,3);C(2,3);C(3,3)];
pC=[C(1,4);C(2,4);C(3,4)];

x = dot(nA, (pB - pA));
y = dot(oA, (pB - pA));
z = dot(aA, (pB - pA));
psi = atan2(dot(oA,aB), dot(nA, aB));
temp = sqrt(dot(nA, aB)^2 + dot(oA, aB)^2);
theta = atan2(temp, dot(aA, aB));
V_r_theta = 1-cos(r*theta);
sin_phi = -sin(psi)*cos(psi)*V_r_theta*dot(nA, nB) + (cos(psi)^2*V_r_theta+cos(theta))*dot(oA, nB) - sin(psi)*sin(theta)*dot(aA, nB);
cos_phi = -sin(psi)*cos(psi)*V_r_theta*dot(nA, oB) + (cos(psi)^2*V_r_theta+cos(theta))*dot(oA, oB) - sin(psi)*sin(theta)*dot(aA, oB);
phi = atan2(sin_phi, cos_phi);


dataA = 1;   % the index of the data of the matrix 
for t=-0.5:sampling_rate:-0.2
    h=(t+0.5)/0.5;
    dx=x*h;
    dy=y*h;
    dz=z*h; 
    dsi=psi;
    dtheta=theta*h;
    dphi=phi*h;   
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    S_psi=sin(psi);
    C_psi=cos(psi);
    S_theta=sin(dtheta);
    C_theta=cos(dtheta);
    V_theta=1-C_theta;
    S_phi=sin(dphi);
    C_phi=cos(dphi);
    
    % find Dr with Dr=Tr*Rar*Ror
    Tr = [1 0 0 dx; 0 1 0 dy; 0 0 1 dz; 0 0 0 1];
    Rar = [S_psi^2*V_theta+C_phi, -S_psi*C_psi*V_theta,C_psi*S_phi, 0;
        -S_psi*C_psi*V_theta,C_psi^2*V_theta+C_phi, S_psi*S_phi, 0;
        -C_psi*S_phi, -S_psi*S_phi, C_phi, 0;
        0, 0, 0, 1];
    Ror = [C_theta, -S_theta, 0, 0;
        S_theta, C_theta, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
    Dr = Tr*Rar*Ror;
    
    pA_B(:,:,dataA)=A*Dr;  %NOAP
    xA_B(:,dataA)=pA_B(1,4,dataA);
    yA_B(:,dataA)=pA_B(2,4,dataA);
    zA_B(:,dataA)=pA_B(3,4,dataA);  
    dataA=dataA+1;
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% (二) 從A'到C'曲線部分的路徑規劃 (-0.2s~0.2s)
%~~~~~~~~~~> xA、yA、zA、psiA、thetaA、phiA <~~~~~~~~~~~~~~~~
%~~~~~~~~~~> xC、yC、zC、psiC、thetaC、phiC <~~~~~~~~~~~~~~~~
                    
A2=pA_B(:,:,dataA-1);% A’的位置
nA2=[A2(1,1);A2(2,1);A2(3,1)];
oA2=[A2(1,2);A2(2,2);A2(3,2)];
aA2=[A2(1,3);A2(2,3);A2(3,3)];
pA2=[A2(1,4);A2(2,4);A2(3,4)];

xA=nB'*(pA2-pB);
yA=oB'*(pA2-pB);
zA=aB'*(pA2-pB);
psiA=atan2(oB'*aA2,nB'*aA2);
thetaA=atan2(sqrt((nB'*aA2)^2+(oB'*aA2)^2),aB'*aA2);
SphiA=-sin(psiA)*cos(psiA)*(1-cos(thetaA))*(nB'*nA2)+((cos(psiA))^2*(1-cos(thetaA))+cos(thetaA))*(oB'*nA2)-sin(psiA)*sin(thetaA)*(aB'*nA2);
CphiA=-sin(psiA)*cos(psiA)*(1-cos(thetaA))*(nB'*oA2)+((cos(psiA))^2*(1-cos(thetaA))+cos(thetaA))*(oB'*oA2)-sin(psiA)*sin(thetaA)*(aB'*oA2);
phiA=atan2(SphiA,CphiA);

xC=nB'*(pC-pB);
yC=oB'*(pC-pB);
zC=aB'*(pC-pB);
psiC=atan2(oB'*aC,nB'*aC);
thetaC=atan2(sqrt((nB'*aC)^2+(oB'*aC)^2),aB'*aC);
SphiC=-sin(psiC)*cos(psiC)*(1-cos(thetaC))*(nB'*nC)+((cos(psiC))^2*(1-cos(thetaC))+cos(thetaC))*(oB'*nC)-sin(psiC)*sin(thetaC)*(aB'*nC);
CphiC=-sin(psiC)*cos(psiC)*(1-cos(thetaC))*(nB'*oC)+((cos(psiC))^2*(1-cos(thetaC))+cos(thetaC))*(oB'*oC)-sin(psiC)*sin(thetaC)*(aB'*oC);
phiC=atan2(SphiC,CphiC);

if abs(psiC-psiA)>pi/2
    psiA=psiA+pi;
    thetaA=-thetaA;
end
% path planing

dataB=1;
for t=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
    h=(t+0.2)/(0.2+0.2);
    dx_B=((xC*0.2/0.5+xA)*(2-h)*h^2-2*xA)*h+xA;
    dy_B=((yC*0.2/0.5+yA)*(2-h)*h^2-2*yA)*h+yA;
    dz_B=((zC*0.2/0.5+zA)*(2-h)*h^2-2*zA)*h+zA;
    dpsi_B=(psiC-psiA)*h+psiA;
    dtheta_B=((thetaC*0.2/0.5+thetaA)*(2-h)*h^2-2*thetaA)*h+thetaA;
    dphi_B=((phiC*0.2/0.5+phiA)*(2-h)*h^2-2*phiA)*h+phiA;
 
    S_psi=sin(dpsi_B);
    C_psi=cos(dpsi_B);
    S_theta=sin(dtheta_B);
    C_theta=cos(dtheta_B);
    V_theta=1-C_theta;
    S_phi=sin(dphi_B);
    C_phi=cos(dphi_B);
    
    Tr = [1 0 0 dx_B; 0 1 0 dy_B; 0 0 1 dz_B; 0 0 0 1];
    Rar = [S_psi^2*V_theta+C_phi, -S_psi*C_psi*V_theta,C_psi*S_phi, 0;
        -S_psi*C_psi*V_theta,C_psi^2*V_theta+C_phi, S_psi*S_phi, 0;
        -C_psi*S_phi, -S_psi*S_phi, C_phi, 0;
        0, 0, 0, 1];
    Ror = [C_theta, -S_theta, 0, 0;
        S_theta, C_theta, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
    Dr_B = Tr*Rar*Ror;                    
  
    p_B(:,:,dataB)=B*Dr_B;
    x_B(:,dataB)=p_B(1,4,dataB);
    y_B(:,dataB)=p_B(2,4,dataB);
    z_B(:,dataB)=p_B(3,4,dataB);  
    dataB=dataB+1;
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% (三) 從C'到C直線部分的路徑規劃 (0.2s~0.5s)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% path planing
dataC=1;
for t=0.2:sampling_rate:0.5
    h=t/0.5;
    dx_C=xC*h;
    dy_C=yC*h;
    dz_C=zC*h;
    dpsi_C=psiC;
    dtheta_C=thetaC*h;
    dphi_C=phiC*h;
 
    S_psi=sin(dpsi_C);
    C_psi=cos(dpsi_C);
    S_theta=sin(dtheta_C);
    C_theta=cos(dtheta_C);
    V_theta=1-C_theta;
    S_phi=sin(dphi_C);
    C_phi=cos(dphi_C);
    
    % find Dr with Dr=Tr*Rar*Ror    
    Tr = [1 0 0 dx_C; 0 1 0 dy_C; 0 0 1 dz_C; 0 0 0 1];
    Rar = [S_psi^2*V_theta+C_phi, -S_psi*C_psi*V_theta,C_psi*S_phi, 0;
        -S_psi*C_psi*V_theta,C_psi^2*V_theta+C_phi, S_psi*S_phi, 0;
        -C_psi*S_phi, -S_psi*S_phi, C_phi, 0;
        0, 0, 0, 1];
    Ror = [C_theta, -S_theta, 0, 0;
        S_theta, C_theta, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
    Dr_C = Tr*Rar*Ror;
    
    p_C(:,:,dataC)=B*Dr_C;
    x_C(:,dataC)=p_C(1,4,dataC);
    y_C(:,dataC)=p_C(2,4,dataC);
    z_C(:,dataC)=p_C(3,4,dataC);  
    dataC=dataC+1;
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 從A到C之x,y,z各軸的變化情形
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
X=[xA_B x_B x_C];
Y=[yA_B y_B y_C];
Z=[zA_B z_B z_C];
t=-0.5:sampling_rate:0.5;
figure(10)
subplot(3,1,1);
plot(t,X);
title('position of x');
grid
subplot(3,1,2);
plot(t,Y);
ylabel('Position(cm)')
title('position of y');
grid
subplot(3,1,3);
plot(t,Z);
title('position of z');
xlabel('Time(s)')
grid


%~~~~~~~~~~~~~~~~~~~~~~~~
% 從A郅C的3D路徑圖
%~~~~~~~~~~~~~~~~~~~~~~~~
figure(20)
plot3(xA_B,yA_B,zA_B,x_B,y_B,z_B,x_C,y_C,z_C);
xlabel('x(cm)');
ylabel('y(cm)');
zlabel('z(cm)');
text(20,30,20,'A(20,30,20)');
text(-10,-20,40,'B(-10,-20,40)');
text(30,40,20,'C(30,40,20)');
grid
title('3D path of Cartesion Motion')

%~~~~~~~~~~~~~~~~~~~~~~~~~~
% 從A到C速度的變化情形
%~~~~~~~~~~~~~~~~~~~~~~~~~~
dt=t(2:501);
dX=diff(X)/sampling_rate;
dY=diff(Y)/sampling_rate;
dZ=diff(Z)/sampling_rate;
figure(30)
subplot(3,1,1);
plot(dt,dX);
title('velocity of x');
grid
subplot(3,1,2);
plot(dt,dY);
title('velocity of y');
grid
subplot(3,1,3);
plot(dt,dZ);
title('velocity of z');
xlabel('Time(s)')
ylabel('Velocity(cm/s)');
grid
%~~~~~~~~~~~~~~~~~~~~~~~~~~
% 從A到C加速度的變化情形
%~~~~~~~~~~~~~~~~~~~~~~~~~~
dt2=t(3:501);
dX2=diff(dX)/sampling_rate;
dY2=diff(dY)/sampling_rate;
dZ2=diff(dZ)/sampling_rate;
figure(40)
subplot(3,1,1);
plot(dt2,dX2);
title('acceleration of x');
grid
subplot(3,1,2);
plot(dt2,dY2);
title('acceleration of y');
grid
subplot(3,1,3);
plot(dt2,dZ2);
title('acceleration of z');
xlabel('Time(s)')
ylabel('Acceleration(cm/s^2)');
grid

%NOAP : pA_B, p_B, p_C

cd(function_path);



size_pA_B = size(pA_B);
theta_pA_B = zeros(size_pA_B(3), 6);
for i = 1 : size_pA_B(3)
    T_input = pA_B(:, :, i);
    T_input(:, 4) = T_input(:, 4)/100;  %公分換公尺
    theta_pA_B(i, :) = kinematics_one_sol(T_input);

   

end

size_p_B = size(p_B);
theta_p_B = zeros(size_p_B(3), 6);
for i = 1 : size_p_B(3)
    T_input = p_B(:, :, i);
    T_input(:, 4) = T_input(:, 4)/100;  %公分換公尺
    theta_p_B(i, :) = kinematics_one_sol(T_input);

end

size_p_C = size(p_C);
theta_p_C = zeros(size_p_C(3), 6);
for i = 1 : size_p_C(3)
    T_input = p_C(:, :, i);
    T_input(:, 4) = T_input(:, 4)/100;  %公分換公尺
    theta_p_C(i, :) = kinematics_one_sol(T_input);

end

theta_all = [theta_pA_B; theta_p_B; theta_p_C];


figure;
subplot(3,2,1);plot(theta_all(:,1));  %joint1
title("Joint 1");
subplot(3,2,2);plot(theta_all(:,2));
title("Joint 2");
subplot(3,2,3);plot(theta_all(:,3));
title("Joint 3");
subplot(3,2,4);plot(theta_all(:,4));
title("Joint 4");
subplot(3,2,5);plot(theta_all(:,5));
title("Joint 5");
subplot(3,2,6);plot(theta_all(:,6));
title("Joint 6");


%角度校正，為了不讓角度差太多
size_theta_all = size(theta_all);
test_theta = 4;
change = 0;

for i = 2:size_theta_all(1)

    if(theta_all(i, test_theta) < 0)
       theta_all(i, test_theta) = theta_all(i, test_theta) + 180;

    end
end
%角度校正，為了不讓角度差太多



figure;
subplot(3,2,1);plot(theta_all(:,1));  %joint1
title("Joint 1");
subplot(3,2,2);plot(theta_all(:,2));
title("Joint 2");
subplot(3,2,3);plot(theta_all(:,3));
title("Joint 3");
subplot(3,2,4);plot(theta_all(:,4));
title("Joint 4");
subplot(3,2,5);plot(theta_all(:,5));
title("Joint 5");
subplot(3,2,6);plot(theta_all(:,6));
title("Joint 6");


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% Joint Move
%{

i=0;
for j1=-0.5:sampling_rate:-0.2
    i=i+1;
    Jtheta=pre(pA_B(:,:,i));    
    p1_theta(:,i)=Jtheta(:,1); % 6*151
end

i=0;
for j2=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
    i=i+1;
    Jtheta=pre(p_B(:,:,i)); 
    p2_theta(:,i)=Jtheta(:,1); %6*201
end

i=0;
for j3=0.2:sampling_rate:0.5
    i=i+1;
    Jtheta=pre(p_C(:,:,i));
    p3_theta(:,i)=Jtheta(:,1); %6*151
end
theta1=[p1_theta(1,:) p2_theta(1,:) p3_theta(1,:)];
theta2=[p1_theta(2,:) p2_theta(2,:) p3_theta(2,:)];
theta3=[p1_theta(3,:) p2_theta(3,:) p3_theta(3,:)];
theta4=[p1_theta(4,:) p2_theta(4,:) p3_theta(4,:)];
theta5=[p1_theta(5,:) p2_theta(5,:) p3_theta(5,:)];
theta6=[p1_theta(6,:) p2_theta(6,:) p3_theta(6,:)];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%check theta4~~6
for i=1:500
    if (abs(theta4(i+1)-theta4(i))>=150)
        theta4(i+1)=theta4(i+1)+180;
        theta6(i+1)=theta6(i+1)-180;
        theta5(i+1)=-theta5(i+1);
    end
end
figure(50)
subplot(3,2,1)
plot(t,theta1)
grid
title('theta1');
ylabel('Angle(。)');
subplot(3,2,2)
plot(t,theta2)
grid
title('theta2');
ylabel('Angle(。)');
subplot(3,2,3)
plot(t,theta3)
grid
title('theta3');
ylabel('Angle(。)');
subplot(3,2,4)
plot(t,theta4)
grid
title('theta4');
ylabel('Angle(。)');
subplot(3,2,5)
plot(t,theta5)
grid
title('theta5');
ylabel('Angle(。)');
subplot(3,2,6)
plot(t,theta6)
grid
title('theta6');
ylabel('Angle(。)');
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% Velocity
t2=t(2:501);
omega1=diff(theta1)/sampling_rate;
omega2=diff(theta2)/sampling_rate;
omega3=diff(theta3)/sampling_rate;
omega4=diff(theta4)/sampling_rate;
omega5=diff(theta5)/sampling_rate;
omega6=diff(theta6)/sampling_rate;
figure(60)
subplot(3,2,1);
plot(t2,omega1);
grid
title('theta1');
ylabel('Angular Velocity');
subplot(3,2,2);
plot(t2,omega2);
grid
title('theta2');
ylabel('Angular Velocity');
subplot(3,2,3);
plot(t2,omega3);
grid
title('theta3');
ylabel('Angular Velocity');
subplot(3,2,4);
plot(t2,omega4);
grid
title('theta4');
ylabel('Angular Velocity');
subplot(3,2,5);
plot(t2,omega5);
grid
title('theta5');
ylabel('Angular Velocity');
subplot(3,2,6);
plot(t2,omega6);
grid
title('theta6');
ylabel('Angular Velocity');
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~% 角加速度
t3=t(3:501);
alpha1=diff(omega1)/sampling_rate;
alpha2=diff(omega2)/sampling_rate;
alpha3=diff(omega3)/sampling_rate;
alpha4=diff(omega4)/sampling_rate;
alpha5=diff(omega5)/sampling_rate;
alpha6=diff(omega6)/sampling_rate;
figure(70)
subplot(3,2,1);
plot(t3,alpha1);
grid
title('theta1');
ylabel('Angular Acceleration');
subplot(3,2,2);
plot(t3,alpha2);
grid
title('theta2');
ylabel('Angular Acceleration');
subplot(3,2,3);
plot(t3,alpha3);
grid
title('theta3');
ylabel('Angular Acceleration');
subplot(3,2,4);
plot(t3,alpha4);
grid
title('theta4');
ylabel('Angular Acceleration');
subplot(3,2,5);
plot(t3,alpha5);
grid
title('theta5');
ylabel('Angular Acceleration');
subplot(3,2,6);
plot(t3,alpha6);
grid
title('theta6');
ylabel('Angular Acceleration');
%}
