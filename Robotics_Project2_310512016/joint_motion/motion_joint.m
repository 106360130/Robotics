
% Stanford type robot manipulator of Joint Motion 
clc
clear all
close all

A = [0 0 -1 40; -1 0 0 -30; 0 1 0 10; 0 0 0 1];
B = [1 0 0 30; 0 -1 0 30; 0 0 -1 20; 0 0 0 1];
C = [0 1 0 40; 0 0 -1 20; -1 0 0 -30; 0 0 0 1];
        

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
                       

  
% linear portion
T = 0.5;
time_point = [-0.5, -0.2, 0.2, 0.5];
delta_B = theta_B - theta_A;
[q_A dq_A ddq_A] = portion_liner(time_point(1:2), sample_rate, T, delta_B, theta_B);
% linear portion


% accelerated portion
t_acc = 0.2;
theta_A2 = theta_A + (theta_B-theta_A)/T*(T-t_acc);
[q_B dq_B ddq_B] = portion_accelerated(time_point(2:3), sample_rate, T, t_acc, theta_A2, theta_B, theta_C);
% accelerated portion



% linear portion
delta_C = theta_C - theta_B;
[q_C dq_C ddq_C] = portion_liner(time_point(3:4), sample_rate, T, delta_C, theta_B);
% linear portion



% angle of joint 1~6
figure;
t = time_point(1) : sample_rate : time_point(4);
t = t + T;
for i = 1:6
    theta = [q_A(i, :) q_B(i, :) q_C(i, :)];                     
    subplot(3, 2, i);plot(t, theta);
    xlabel('Time (sec)');
    ylabel('Angle ( degree)');
    title(['Joint ', num2str(i)]);
    grid;

end
% angle of joint 1~6


% veloctity of joint 1~6
figure;
for i = 1:6
    subplot(3, 2, i)
    plot(t, [dq_A(i, :) dq_B(i, :) dq_C(i, :)]);  
    xlabel('Time (sec)');
    ylabel('Angular Velocity (degree/s)');
    title(['Joint ', num2str(i)]);
    grid

end
% veloctity of joint 1~6


% accleration of joint 1~6
figure;
for i = 1:6
    subplot(3, 2, i)
    plot(t, [ddq_A(i, :) ddq_B(i, :) ddq_C(i, :)]);
    xlabel('Time (sec)');
    ylabel('Angular Acceleration (degree/s^2)');
    title(['joint ', num2str(i)]);
    grid

end
% accleration of joint 1~6



% forward kinematics


% path_1
t =  time_point(1) : sample_rate : time_point(2);
path_1 = zeros(length(t), 3);
i=1;
for t =  time_point(1) : sample_rate : time_point(2) 
    %qA(:,i)'
    temp_position = kinematics_forward(q_A(:, i)');
    path_1(i, :) = temp_position(1:3, 4)';


    i = i+1;
end
% path_1

% path_2
t = ( time_point(2) + sample_rate) : sample_rate : ( time_point(3) - sample_rate);
path_2 = zeros(length(t), 3);
i=1;
for t = ( time_point(2) + sample_rate) : sample_rate : ( time_point(3) - sample_rate)
    temp_position = kinematics_forward(q_B(:,i)');
    path_2(i, :) = temp_position(1:3, 4)';


    i = i+1;
end
% path_2

% path_3
t =  time_point(3)  : sample_rate : time_point(4);
path_3 = zeros(length(t), 3);
i=1;
for t =  time_point(3)  : sample_rate : time_point(4)
    temp_position = kinematics_forward(q_C(:,i)');
    path_3(i, :) = temp_position(1:3, 4)';


    i = i+1;
end
% path_3
% forward kinematics


% position of x, y, z
position_all = [path_1; path_2; path_3];
figure;
plot3(position_all(:, 1), position_all(:, 2), position_all(:, 3));
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
title('3D path of Joint Motion');
grid


t = time_point(1) : sample_rate : time_point(4);
t = t + T;
text_axis = ['X' 'Y' 'Z'];

figure;
for i = 1:3
    subplot(3, 1, i);plot(t, position_all(:, i));
    xlabel('Time (sec)')
    ylabel('Position (m)')
    title(text_axis(i));
    grid

end
% position of x, y, z


% velocity of x, y, z
size_position_all = size(position_all);
velocity_all = zeros(size_position_all(1)-1, size_position_all(2));
for i = 1:3
    velocity_all(:, i) = diff(position_all(:, i));
end

t = linspace(0, 1, size_position_all(1)-1);

figure;
for i = 1 : 3
    subplot(3, 1, i); plot(t, velocity_all(:, i));
    xlabel('Time(sec)');
    ylabel('Velocity(m/s)');
    title(text_axis(i));
    grid

end
% velocity of x, y, z


% accleration of x, y, z
size_velocity_all = size(velocity_all);
acceleration_all = zeros(size_velocity_all(1)-1, size_velocity_all(2));

for i = 1:3
    acceleration_all(:, i) = diff(velocity_all(:, i));

end

t = linspace(0, 1, size_velocity_all(1)-1);
figure;
for i = 1:3
    subplot(3, 1, i); plot(t, acceleration_all(:, i));
    xlabel('Time (sec)')
    ylabel('Acceleration (m/s^2)')
    title(text_axis(i));
    grid
    
end
% accleration of x, y, z

