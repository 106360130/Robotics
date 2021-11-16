clear all;
close all;


%% Forward Kinematics
% theta_test = [90, 99, -119, -10, 10, 0];
theta_input = input(['Enter the joint variable : \n' ...
    '[theta1   theta2   theta3   theta4   theta5   theta6]\n']);

T = kinematics_forward(theta_input);  %forward kinematics
euler_angles = euler_angle(T);

%print result
fprintf('\n\n[\t\t n \t\t a \t\t o \t\t p]');
T

fprintf('[\t\t x \t\t y \t\t z \t\t phi \t\t theta \t\t psi]');
[T(1, 4) T(2, 4) T(3, 4) T(4, 4) euler_angles']
%print result


%計算係數用
%{
a = [0.12, 0.25, 0.26, 0, 0, 0];
d = [0, 0, 0, 0, 0, 0];
alpha=[-90, 0, 0, -90, 90, 0];
i = 6;
T_result = T_coe(d(i), a(i), alpha(i));
%}
%計算係數用



%% Inverse Kinematics
%[0.7936 -0 -0.9848 0; 0.8529 0.5 0.1504 0.3252; 0.4924 -0.866 0.0868 -0.158; 0 0 0 1] 
%[-0.7071 -0 -0.7071 0; 0.5 0.7071 -0.5 0.37; 0.5 -0.7071 -0.5 0.26; 0 0 0 1]
T_input = input(['Enter the Cartestian point [n o a p]\n' ...
    ['[nx   ox   ax   px\n' ...
    [' ny   oy   ay   px\n' ...
    [' nz   oz   az   pz\n' ...
     '  0    0    0   1]\n']]]]);

%inverse kinematics
theta_inverse_1 = kinematics_inverse_1(T_input)*(180/pi);
theta_inverse_2 = kinematics_inverse_2(T_input)*(180/pi);
theta_inverse_3 = kinematics_inverse_3(T_input)*(180/pi);
theta_inverse_4 = kinematics_inverse_4(T_input)*(180/pi);
theta_inverse_5 = kinematics_inverse_5(T_input)*(180/pi);
theta_inverse_6 = kinematics_inverse_6(T_input)*(180/pi);
theta_inverse_7 = kinematics_inverse_7(T_input)*(180/pi);
theta_inverse_8 = kinematics_inverse_8(T_input)*(180/pi);

theta_output = [theta_inverse_1'; theta_inverse_2'; theta_inverse_3'; theta_inverse_4';
    theta_inverse_5'; theta_inverse_6'; theta_inverse_7'; theta_inverse_8'];
%inverse kinematics

%check angels
error_output = " ";
angle_limit = [-150, 150; -30, 100; -120, 0; -110, 110; -180, 180; -180, 180];
for i = 1:8

    temp = " ";
    for j = 1:6
        if(theta_output(i, j) < angle_limit(j, 1) || theta_output(i, j) > angle_limit(j, 2))
            temp = temp + string(j) + " ";
        end
    end

    error_output = [error_output; temp];
 
end
%check angels


%print result
fprintf('\n\nInverse Kinematic : ');

for i = 1:8
    fprintf('\n\n[theta1  theta2  theta3  theta4  theta5  theta6]\n');
    
    if(error_output(i+1, 1) == " 1 2 3 4 5 6 ")
       fprintf('    out of reach\n');

    elseif(error_output(i+1, 1) ~= " ")
       theta_output(i, :)
       fprintf('    joint %s out of range\n', error_output(i+1, 1)); 
      
    else
       theta_output(i, :)

    end
end
%print result


