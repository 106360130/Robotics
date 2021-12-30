
% Stanford type robot manipulator of Joint Motion 
clc
clear all
close all




A = [0 0 -1 40; -1 0 0 -30; 0 1 0 10; 0 0 0 1];
B = [1 0 0 30; 0 -1 0 30; 0 0 -1 20; 0 0 0 1];
C = [0 1 0 40; 0 0 -1 20; -1 0 0 -30; 0 0 0 1];


%cm -> m
A(:, 4) =  A(:, 4)/100;
B(:, 4) =  B(:, 4)/100;
C(:, 4) =  C(:, 4)/100;

thetaA = A;
thetaB = B;
thetaC = C;
%cm -> m


s_rate = 0.002;
                       
i=1;
  
% From A to A' angle,angle_acceleration and acceleration
for sample_time=-0.5:s_rate:-0.2

    qA(:, :, i)=(thetaB-thetaA)/0.5*(sample_time+0.5)+thetaA;                
    dqA(:,:,i)=(thetaB-thetaA)/0.5;
    
    ddqA(:,i)=[0;0;0;0;0;0];
    i=i+1;
end

% Path 2 joint angle variation
% A2 means A' 
   
thetaA2=thetaA+(thetaB-thetaA)/0.5*(0.5-0.2);                        
dB=thetaA2-thetaB;

dC=thetaC-thetaB;
i=1;

  % From A to B angle,angle_acceleration and acceleration
for sample_time=(-0.2+s_rate):s_rate:(0.2-s_rate)
    h=(sample_time+0.2)/0.4;
    qB(:,:,i)=((dC*0.2/0.5+dB)*(2-h)*h^2-2*dB)*h+thetaB+dB;  
    
    dqB(:,:,i)=((dC*0.2/0.5+dB)*(1.5-h)*2*h^2-dB)/0.2;
    ddqB(:,:,i)=(dC*0.2/0.5+dB)*(1-h)*3*h/0.2^2;
    i=i+1;
end

% Path 3 joint angle variation
i=1;
for sample_time=0.2:s_rate:0.5
    h=sample_time/0.5;
    qC(:,:,i)=dC*h+thetaB;
    
    dqC(:,:,i)=dC/0.5;
    ddqC(:,i)=[0;0;0;0;0;0];
    i=i+1;
end


% From A to C angle variation at joint veiw 

figure(1)
sample_time=-0.5:s_rate:0.5;

size_qA = size(qA);
P_A = zeros(size_qA(3), 3);
for i = 1:size_qA(3)
    P_A(i, :) = qA(1:3, 4, i)';
end

size_qB = size(qB);
P_B = zeros(size_qB(3), 3);
for i = 1:size_qB(3)
    P_B(i, :) = qB(1:3, 4, i)';
end

size_qC = size(qC);
P_C = zeros(size_qC(3), 3);
for i = 1:size_qC(3)
    P_C(i, :) = qC(1:3, 4, i)';
end

P_final = [P_A; P_B; P_C];
figure;
subplot(3,1,1);plot(P_final(:,1));
subplot(3,1,2);plot(P_final(:,2));
subplot(3,1,3);plot(P_final(:,3));

function_path = "C:\Users\RTES\Desktop\Jakob\Robotics\Project\project_2\Reference\inverse_kinematics_2";
cd(function_path);
theta_A = zeros(size_qA(3), 6);
for i = 1:size_qA(3)
    theta_A(i, :) = kinematics_one_sol(qA(:, :, i));
end

theta_B = zeros(size_qB(3), 6);
for i = 1:size_qB(3)
    theta_B(i, :) = kinematics_one_sol(qB(:, :, i));
end

theta_C = zeros(size_qC(3), 6);
for i = 1:size_qC(3)
    theta_C(i, :) = kinematics_one_sol(qC(:, :, i));
end

theta_all = [theta_A; theta_B; theta_C];
size_theta_all = size(theta_all);
time_x = linspace(0,1,size_theta_all(1));
figure;
for i = 1 :6
    subplot(3,2,i);plot(time_x, theta_all(:,i));
    title("Joint " + i)
end
%theta
%{

theta_1=[qA(1,:) qB(1,:) qC(1,:)];                     
subplot(3,2,1);
plot(sample_time,theta_1);
grid
title('joint1');
ylabel('Angle(degree)');

theta_2=[qA(2,:) qB(2,:) qC(2,:)];                    
subplot(3,2,2);
plot(sample_time,theta_2);
grid
title('joint2');
ylabel('Angle(degree)');

theta_3=[qA(3,:) qB(3,:) qC(3,:)];                     
subplot(3,2,3);
plot(sample_time,theta_3);
grid
title('joint3');
ylabel('distance(cm)');

theta_4=[qA(4,:) qB(4,:) qC(4,:)];                     
subplot(3,2,4);
plot(sample_time,theta_4);
grid
title('joint4');
ylabel('Angle(degree)');

theta_5=[qA(5,:) qB(5,:) qC(5,:)];                      
subplot(3,2,5);
plot(sample_time,theta_5);
grid
title('joint5');
ylabel('Angle(degree)');

theta_6=[qA(6,:) qB(6,:) qC(6,:)];                     
subplot(3,2,6);
plot(sample_time,theta_6);
grid
title('joint6');
ylabel('Angle(degree)');
%theta


% From A to C angle veloctity variation at joint veiw 

figure(2)
subplot(3,2,1)
plot(sample_time,[dqA(1,:) dqB(1,:) dqC(1,:)]);  
grid
title('joint1');
ylabel('Angular Velocity');

subplot(3,2,2)
plot(sample_time,[dqA(2,:) dqB(2,:) dqC(2,:)]);   
grid
title('joint2');
ylabel('Angular Velocity');

subplot(3,2,3)
plot(sample_time,[dqA(3,:) dqB(3,:) dqC(3,:)]); 
grid
title('joint3');
ylabel('Velocity');

subplot(3,2,4)
plot(sample_time,[dqA(4,:) dqB(4,:) dqC(4,:)]);    
grid
title('joint4');
ylabel('Angular Velocity');

subplot(3,2,5)
plot(sample_time,[dqA(5,:) dqB(5,:) dqC(5,:)]);  
grid
title('joint5');
ylabel('Angular Velocity');

subplot(3,2,6)
plot(sample_time,[dqA(6,:) dqB(6,:) dqC(6,:)]);   
grid
title('joint6');
ylabel('Angular Velocity');

% From A to C angle acceleration variation at joint veiw 

figure(3)
subplot(3,2,1)
plot(sample_time,[ddqA(1,:) ddqB(1,:) ddqC(1,:)]);
grid
title('joint1');
ylabel('Angular Acceleration');

subplot(3,2,2)
plot(sample_time,[ddqA(2,:) ddqB(2,:) ddqC(2,:)]);
grid
title('joint2');
ylabel('Angular Acceleration');

subplot(3,2,3)
plot(sample_time,[ddqA(3,:) ddqB(3,:) ddqC(3,:)]);
grid
title('joint3');
ylabel('Acceleration');

subplot(3,2,4)
plot(sample_time,[ddqA(4,:) ddqB(4,:) ddqC(4,:)]);
grid
title('joint4');
ylabel('Angular Acceleration');

subplot(3,2,5)
plot(sample_time,[ddqA(5,:) ddqB(5,:) ddqC(5,:)]);
grid
title('joint5');
ylabel('Angular Acceleration');

subplot(3,2,6)
plot(sample_time,[ddqA(6,:) ddqB(6,:) ddqC(6,:)]);
grid
title('joint6');
ylabel('Angular Acceleration');

% Use Stanford type robot manipulator of Kinematics to convert Joint Path to Cartestion Path
% Path 1

%forward kinematics
kinematics_path = "C:\Users\RTES\Desktop\Jakob\Robotics\Project\project_2\Reference\kinematics_forward";
cd(kinematics_path);
i=1;

for t1=-0.5:s_rate:-0.2
    %qA(:,i)'
    p1=kinematics_forward(qA(:,i)');                   
    x1(i)=p1(1,4);
    y1(i)=p1(2,4);
    z1(i)=p1(3,4);
%     Cphi_1(i)=p1(4,1);
%     Ctheta1(i)=p1(5,1);
%     Cpsi_1(i)=p1(6,1);
    i=i+1;
end

% Path2

i=1;
for t2=(-0.2+s_rate):s_rate:(0.2-s_rate)
    p2=kinematics_forward(qB(:,i)');
    x2(i)=p2(1,4);
    y2(i)=p2(2,4);
    z2(i)=p2(3,4);
%     Cphi2(i)=p2(4,1);
%     Ctheta2(i)=p2(5,1);
%     Cpsi2(i)=p2(6,1);
    i=i+1;
end

% Path3

i=1;
for t3=0.2:s_rate:0.5
    %Kinematics convert
    p3=kinematics_forward(qC(:,i)');
    x3(i)=p3(1,4);
    y3(i)=p3(2,4);
    z3(i)=p3(3,4);
%     Cphi3(i)=p3(4,1);
%     Ctheta3(i)=p3(5,1);
%     Cpsi3(i)=p3(6,1);
    i=i+1;
end
%forward kinematics

%  Use Joint Motion to convert Cartesian of 3D This is figure 4
figure(4)
plot3(x1,y1,z1,x2,y2,z2,x3,y3,z3);
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
text(20,10,-10,'A(20,10,-10)');
text(20,-5,10,'B(20,-5,10)');
text(-10,15,25,'C(-10,15,25)');
grid
title('3D path of Joint Motion')


% x,y,z position variation as time varies This is figure5
figure(5)
X=[x1 x2 x3];
Y=[y1 y2 y3];
Z=[z1 z2 z3];
sample_time=-0.5:s_rate:0.5;
subplot(3,1,1);
plot(sample_time,X);
grid
title('position of x');
subplot(3,1,2);
plot(sample_time,Y);
grid
title('position of y');
ylabel('Position(m)')
subplot(3,1,3);
plot(sample_time,Z);
grid
title('position of z');
xlabel('Time(s)')

% x,y,z velocity variation This is figure6  


tv=sample_time(2:length(sample_time));
% dX=diff(X)/s_rate;
% dY=diff(Y)/s_rate;
% dZ=diff(Z)/s_rate;
dX=diff(X);
dY=diff(Y);
dZ=diff(Z);
figure(6)
subplot(3,1,1);
plot(tv,dX);
grid
title('velocity of x');
subplot(3,1,2);
plot(tv,dY);
grid
title('velocity of y');
ylabel('Velocity(m/s)');
subplot(3,1,3);
plot(tv,dZ);
grid
title('velocity of z');
xlabel('Time(s)')

% x,y,z accleration variation This is figure7

ta=sample_time(3:length(sample_time));
% dX2=diff(dX)/s_rate;
% dY2=diff(dY)/s_rate;
% dZ2=diff(dZ)/s_rate;
dX2=diff(dX);
dY2=diff(dY);
dZ2=diff(dZ);
figure(7)
subplot(3,1,1);
plot(ta,dX2);
grid
title('acceleration of x');
subplot(3,1,2);
plot(ta,dY2);
grid
title('acceleration of y');
ylabel('Acceleration(m/s^2)');
subplot(3,1,3);
plot(ta,dZ2);
grid
title('acceleration of z');
xlabel('Time(s)')

% phi, theta, psi variation as time varies This is figure8
%{
figure(8)
PHI=[Cphi_1 Cphi2 Cphi3];
THETA=[Ctheta1 Ctheta2 Ctheta3];
PSI=[Cpsi_1 Cpsi2 Cpsi3];
sample_time=-0.5:s_rate:0.5;
subplot(3,1,1);
plot(sample_time,PHI);
grid
title('angle of phi');
subplot(3,1,2);
plot(sample_time,THETA);
grid
title('angle of theta');
ylabel('angle(。)')
subplot(3,1,3);
plot(sample_time,PSI);
grid
title('angle of psi');
xlabel('Time(s)')

% phi, theta, psi veloctiy variation as time varies This is figure9
tv=sample_time(2:1001);
dPHI=diff(PHI)/s_rate;
dTHETA=diff(THETA)/s_rate;
dPSI=diff(PSI)/s_rate;
figure(9)
subplot(3,1,1);
plot(tv,dPHI);
grid
title('angular velocity of phi');
subplot(3,1,2);
plot(tv,dTHETA);
grid
title('angular velocity of theta');
ylabel('angular velocity(degree/s)');
subplot(3,1,3);
plot(tv,dPSI);
grid
title('angular velocity of psi');
xlabel('Time(s)')

% phi, theta, psi acceleration variation This is figure10
ta=sample_time(3:1001);
dPHI2=diff(dPHI)/s_rate;
dTHETA2=diff(dTHETA)/s_rate;
dPSI2=diff(dPSI)/s_rate;
figure(10)
subplot(3,1,1);
plot(ta,dPHI2);
grid
title('acceleration of phi');
subplot(3,1,2);
plot(ta,dTHETA2);
grid
title('angular acceleration of theta');
ylabel('angular acceleration(degree/s^2)');
subplot(3,1,3);
plot(ta,dPSI2);
grid
title('angular acceleration of psi');
xlabel('Time(s)')
%}
%}


%Functions
function [Jtheta] = IK(T6)


dtor = pi/180;  %degree to rad
rtod = 180/pi;  %rad to degree
d2 = 6.375;


%inverse kinematic
    
nx=T6(1,1);
ny=T6(2,1);
nz=T6(3,1);

ox=T6(1,2);
oy=T6(2,2);
oz=T6(3,2);

ax=T6(1,3);
ay=T6(2,3);
az=T6(3,3);

px=T6(1,4);
py=T6(2,4);
pz=T6(3,4);
   
    %theta1
    r = sqrt((px)^2+(py)^2);   
    t11 = atan2(py,px)-atan2(d2,sqrt((r)^2-(d2)^2));  
    t12 = atan2(py,px)+atan2(d2,sqrt((r)^2-(d2)^2)); 
       
    if abs(t12*rtod)<=120
    t1 = t12+pi;
    dm=2;
    end
    
    if  abs(t11*rtod)<=120
    t1 = t11;  
    dm=1;
    end
    
    if  abs(t11*rtod)>120
    t1 = t11+pi;  
    dm=1;
    end
    

   %fprintf('t1= %f  \n',t1*rtod);
    s1 = sin(t1);
    c1 = cos(t1);
    %theta2
    t2 = atan2(c1*px+s1*py,pz);
    s2 = sin(t2);
    c2 = cos(t2);
    %distance3
    d3 = s2*(c1*px+s1*py)+c2*pz;
    %theta4
    t44 = atan2(-s1*ax+c1*ay ,c2*(c1*ax+s1*ay)-s2*az);
    while dm == 1
    t4=t44;
    break 
    end
    while dm == 2
    t4=t44-pi;
    break 
    end
    s4 = sin(t4);
    c4 = cos(t4);
    %theta5
    t5 = atan2(c4*(c2*(c1*ax+s1*ay)-s2*az)+s4*(-s1*ax+c1*ay),s2*(c1*ax+s1*ay)+c2*az);    
    s4 = sin(t4);
    c4 = cos(t4);
    %theta6
    s5 = sin(t5);
    c5 = cos(t5);
    coso = c1*ox + s1*oy;
    soco = -s1*ox + c1*oy;
    so = s2*oz;
    co = c2*oz;
    t6 = atan2(-c5*(c4*(c2*coso-so)+s4*soco)+s5*(s2*coso+co), -s4*(c2*coso-so)+c4*soco );
    
    while abs(t11*rtod)>160
   % fprintf('Theta1 can not enable\n');
    break 
    end
    while abs(t2*rtod)>125
   %     fprintf('Theta2 can not enable\n');
    break 
    end
    while abs(t4*rtod)>140
   % fprintf('Theta4 can not enable\n');
    break 
    end
    while abs(t5*rtod)>100
    %fprintf('Theta5 can not enable\n');
    break 
    end
    while abs(t6*rtod)>260
   % fprintf('Theta6 can not enable\n');
    break 
    end
    
    %fprintf('(theta1,theta2,distance3,theta4,theta5,theta6)=(%f ,%f ,%f ,%f ,%f ,%f) \n',t1*rtod,t2*rtod,d3,t4*rtod,t5*rtod,t6*rtod);
    
    
    theta=[t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod];

Jtheta=theta(1,:)';
end

function [P] = kinematics(joint_varaible)
dtor = pi/180;  %degree to rad
rtod = 180/pi;  %rad to degree
d2 = 1;

%forward kinematic

    joint=joint_varaible;
    t1=joint(1,1);
    t2=joint(1,2);
    d3=joint(1,3);
    t4=joint(1,4);
    t5=joint(1,5);
    t6=joint(1,6);
    
    A1 = [cos(t1*dtor)  0  -sin(t1*dtor)  0 ;
          sin(t1*dtor)  0   cos(t1*dtor)  0 ;
                  0 -1 0 0 ;
                  0 0  0 1];
              
    A2 = [cos(t2*dtor)  0   sin(t2*dtor)  0 ;
          sin(t2*dtor)  0  -cos(t2*dtor)  0 ;
                  0 1 0 d2 ;
                  0 0 0 1 ];
              
    A3 = [   1  0  0  0 ;
             0  1  0  0 ;
             0  0  1  d3 ;
             0  0  0  1 ];
              
    A4 = [cos(t4*dtor)  0  -sin(t4*dtor)  0 ;
          sin(t4*dtor)  0   cos(t4*dtor)  0 ;
                  0 -1 0 0 ;
                  0  0 0 1 ];
        
    A5 = [cos(t5*dtor)  0   sin(t5*dtor)  0 ;
          sin(t5*dtor)  0  -cos(t5*dtor)  0 ;
                  0 1 0 0 ;
                  0 0 0 1 ];
              
    A6 = [cos(t6*dtor)  -sin(t6*dtor)  0  0 ;
          sin(t6*dtor)   cos(t6*dtor)  0  0 ;
                  0 0 1 0 ;
                  0 0 0 1 ];
 
    T6 = A1*A2*A3*A4*A5*A6;
    nx = T6(1,1);
    ny = T6(2,1);
    nz = T6(3,1);  
    ox = T6(1,2);
    oy = T6(2,2);
    oz = T6(3,2); 
    ax = T6(1,3);
    ay = T6(2,3);
    az = T6(3,3); 
    px = T6(1,4);
    py = T6(2,4);
    pz = T6(3,4); 
    
    phi = atan2(ay,ax)*rtod;
    theta = atan2(sqrt((ax)^2+(ay)^2),az)*rtod;
    psi = atan2(oz,-nz)*rtod;
    
    while abs(t1)>160
        break 
    end
    while abs(t2)>125
    break 
    end
    while abs(d3)>30
    break 
    end
    while abs(t4)>140
    break 
    end
    while abs(t5)>100
    break 
    end
    while abs(t6)>260
    break 
    end 

P = [px, py, pz, phi, theta, psi]';

end
%Functions