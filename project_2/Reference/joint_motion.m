
% Stanford type robot manipulator of Joint Motion 
clc
clear all
close all

function_path = "C:\Users\RTES\Desktop\Jakob\Robotics\Project\project_2\Reference\inverse_kinematics";


A = [0 0 -1 40; -1 0 0 -30; 0 1 0 10; 0 0 0 1];
B = [1 0 0 30; 0 -1 0 30; 0 0 -1 20; 0 0 0 1];
C = [0 1 0 40; 0 0 -1 20; -1 0 0 -30; 0 0 0 1];
        
cd(function_path);

%Inverse kinematics
A(:, 4) =  A(:, 4)/100;
B(:, 4) =  B(:, 4)/100;
C(:, 4) =  C(:, 4)/100;

theta_A = kinematics_one_sol(A);
theta_B = kinematics_one_sol(B);
theta_C = kinematics_one_sol(C);

theta_A = theta_A';
theta_B = theta_B';
theta_C = theta_C';
%Inverse kinematics

sample_rate = 0.002;
                       
i = 1;
  
% From A to A' angle,angle_acceleration and acceleration
% linear portion
T = 0.5;
time_point = [-0.5, -0.2, 0.2, 0.5];
delta_B = theta_B - theta_A;
[q_A dq_A ddq_A] = portion_liner(time_point(1:2), sample_rate, T, delta_B, theta_B);
% linear portion

% Path 2 joint angle variation
% A2 means A' 
% From A to B angle,angle_acceleration and acceleration
% accelerated portion
t_acc = 0.2;
theta_A2 = theta_A + (theta_B-theta_A)/T*(T-t_acc);
[q_B dq_B ddq_B] = portion_accelerated(time_point(2:3), sample_rate, T, t_acc, theta_A2, theta_B, theta_C);
% accelerated portion

% Path 3 joint angle variation
% linear portion
delta_C = theta_C - theta_B;
[q_C dq_C ddq_C] = portion_liner(time_point(3:4), sample_rate, T, delta_C, theta_B);
% linear portion


% From A to C angle variation at joint veiw 

figure;
t = time_point(1) : sample_rate : time_point(4);

for i = 1:6
    theta = [q_A(i, :) q_B(i, :) q_C(i, :)];                     
    subplot(3, 2, i);
    plot(t, theta);
    grid
    title(['Joint ', num2str(i)]);
    ylabel('Angle(degree)');

end


% From A to C angle veloctity variation at joint veiw 

figure;
for i = 1:6
    subplot(3, 2, i)
    plot(t, [dq_A(i, :) dq_B(i, :) dq_C(i, :)]);  
    grid
    title(['joint ', num2str(i)]);
    ylabel('Angular Velocity');

end


% From A to C angle acceleration variation at joint veiw 
figure;
for i = 1:6
    subplot(3, 2, i)
    plot(t, [ddq_A(i, :) ddq_B(i, :) ddq_C(i, :)]);
    grid
    title(['joint ', num2str(i)]);
    ylabel('Angular Acceleration');

end



% Use Stanford type robot manipulator of Kinematics to convert Joint Path to Cartestion Path
% Path 1

%forward kinematics
kinematics_path = "C:\Users\RTES\Desktop\Jakob\Robotics\Project\project_2\Reference\kinematics_forward";
cd(kinematics_path);

i=1;
for t1 =  time_point(1) : sample_rate : time_point(2) 
    %qA(:,i)'
    p1 = kinematics_forward(q_A(:, i)');                   
    x1(i) = p1(1, 4);
    y1(i) = p1(2, 4);
    z1(i) = p1(3, 4);

    i = i+1;
end

% Path2

i=1;
for t2 = ( time_point(2) + sample_rate) : sample_rate : ( time_point(3) - sample_rate)
    p2 = kinematics_forward(q_B(:,i)');
    x2(i) = p2(1, 4);
    y2(i) = p2(2, 4);
    z2(i) = p2(3, 4);

    i = i+1;
end

% Path3

i=1;
for t3 =  time_point(3)  : sample_rate : time_point(4)
    %Kinematics convert
    p3 = kinematics_forward(q_C(:,i)');
    x3(i) = p3(1, 4);
    y3(i) = p3(2, 4);
    z3(i) = p3(3, 4);

    i = i+1;
end
%forward kinematics

%  Use Joint Motion to convert Cartesian of 3D This is figure 4
figure;
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
figure;
X=[x1 x2 x3];
Y=[y1 y2 y3];
Z=[z1 z2 z3];
t=-0.5:sample_rate:0.5;
subplot(3,1,1);
plot(t,X);
grid
title('position of x');
subplot(3,1,2);
plot(t,Y);
grid
title('position of y');
ylabel('Position(m)')
subplot(3,1,3);
plot(t,Z);
grid
title('position of z');
xlabel('Time(s)')

% x,y,z velocity variation This is figure6  


tv=t(2:length(t));
% dX=diff(X)/s_rate;
% dY=diff(Y)/s_rate;
% dZ=diff(Z)/s_rate;
dX=diff(X);
dY=diff(Y);
dZ=diff(Z);
figure;
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

ta=t(3:length(t));
% dX2=diff(dX)/s_rate;
% dY2=diff(dY)/s_rate;
% dZ2=diff(dZ)/s_rate;
dX2=diff(dX);
dY2=diff(dY);
dZ2=diff(dZ);
figure;
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

