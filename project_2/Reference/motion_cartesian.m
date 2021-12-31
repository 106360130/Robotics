
% Stanford type robot manipulator of Joint Motion 
clc
clear all
close all

function_path = "C:\Users\RTES\Desktop\Jakob\Robotics\Project\project_2\Reference\inverse_kinematics_2";

A = [0 0 -1 40; -1 0 0 -30; 0 1 0 10; 0 0 0 1];
B = [1 0 0 30; 0 -1 0 30; 0 0 -1 20; 0 0 0 1];
C = [0 1 0 40; 0 0 -1 20; -1 0 0 -30; 0 0 0 1];
        
cd(function_path);


%cm -> m
A(:, 4) =  A(:, 4)/100;
B(:, 4) =  B(:, 4)/100;
C(:, 4) =  C(:, 4)/100;

noap_A = A;
noap_B = B;
noap_C = C;
%cm -> m

sample_rate = 0.002;
                         
% linear portion
T = 0.5;
time_point = [-0.5, -0.2, 0.2, 0.5];
delta_B = noap_B - noap_A;
[q_A dq_A ddq_A] = portion_liner(time_point(1:2), sample_rate, T, delta_B, noap_B);
% linear portion


% accelerated portion
t_acc = 0.2;
theta_A2 = noap_A + (noap_B-noap_A)/T*(T-t_acc);
[q_B dq_B ddq_B] = portion_accelerated(time_point(2:3), sample_rate, T, t_acc, theta_A2, noap_B, noap_C);
% accelerated portion



% linear portion
delta_C = noap_C - noap_B;
[q_C dq_C ddq_C] = portion_liner(time_point(3:4), sample_rate, T, delta_C, noap_B);
% linear portion

% position of x, y, z
size_q_A = size(q_A);
path_1 = zeros(size_q_A(3), 3);
for i = 1 : size_q_A(3)
    path_1(i, :) = q_A(1:3, 4, i)';
end

size_q_B = size(q_B);
path_2 = zeros(size_q_B(3), 3);
for i = 1 : size_q_B(3)
    path_2(i, :) = q_B(1:3, 4, i)';
end

size_q_C = size(q_C);
path_3 = zeros(size_q_C(3), 3);
for i = 1 : size_q_C(3)
    path_3(i, :) = q_C(1:3, 4, i)';
end

position_all = [path_1; path_2; path_3];
figure;
plot3(position_all(:, 1), position_all(:, 2), position_all(:, 3));
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
title('3D path of Cartesian Motion');
grid


t = time_point(1) : sample_rate : time_point(4);
t = t + T;
text_axis = ['X' 'Y' 'Z'];

figure;
for i = 1:3
    subplot(3, 1, i);plot(t, position_all(:, i));
    xlabel('Time(sec)')
    ylabel('Position(m)')
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
    xlabel('Time(s)')
    ylabel('Acceleration(m/s^2)')
    title(text_axis(i));
    grid
    
end
% accleration of x, y, z

% inverse kinematics
function_path = "C:\Users\RTES\Desktop\Jakob\Robotics\Project\project_2\Reference\inverse_kinematics_2";
cd(function_path);
theta_A = zeros(size_q_A(3), 6);
for i = 1:size_q_A(3)
    theta_A(i, :) = kinematics_one_sol(q_A(:, :, i));
end

theta_B = zeros(size_q_B(3), 6);
for i = 1:size_q_B(3)
    theta_B(i, :) = kinematics_one_sol(q_B(:, :, i));
end

theta_C = zeros(size_q_C(3), 6);
for i = 1:size_q_C(3)
    theta_C(i, :) = kinematics_one_sol(q_C(:, :, i));
end
% inverse kinematics

% angle of joint 1~6
t = time_point(1) : sample_rate : time_point(4);
t = t + T;
theta_all = [theta_A; theta_B; theta_C];  

figure;
for i = 1:6
    subplot(3, 2, i);plot(t, theta_all(:, i));
    xlabel('Time(sec)');
    ylabel('Angle(degree)');
    title(['Joint ', num2str(i)]);
    grid;

end
% angle of joint 1~6

% velocity of joint 1~6
size_theta_all = size(theta_all);
velocity_theta = zeros(size_theta_all(1)-1, 6);
for i = 1:6
    velocity_theta(:, i) = diff(theta_all(:, i));
end

t = linspace(0, 1, size_theta_all(1)-1);

figure;
for i = 1:6
    subplot(3, 2, i);plot(t, velocity_theta(:, i));
    xlabel('Time(sec)');
    ylabel('Angular Velocity(degree/s)');
    title(['Joint ', num2str(i)]);
    grid;

end
% velocity of joint 1~6

% accleration of joint 1~6
size_velocity_theta = size(velocity_theta);
accleration_theta = zeros(size_velocity_theta(1)-1, 6);
for i = 1:6
    accleration_theta(:, i) = diff(velocity_theta(:, i));
end

t = linspace(0, 1, size_velocity_theta(1)-1);

figure;
for i = 1:6
    subplot(3, 2, i);plot(t, accleration_theta(:, i));
    xlabel('Time(sec)');
    ylabel('Angular Acceleration(degree/s^2)');
    title(['Joint ', num2str(i)]);
    grid;

end
% accleration of joint 1~6
