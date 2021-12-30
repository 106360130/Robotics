function theta_one_sol = kinematics_one_sol(T_input)

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
angle_limit = [-150, 150; -30, 100; -120, 0; -110, 110; -180, 180; -180, 180];
theta_one_sol = [];
for i = 1:8
    
    correct = 1;
    for j = 1:6
        if(angle_limit(j, 1) <= theta_output(i, j) && theta_output(i, j) <= angle_limit(j, 2))
            correct = correct*1;
        else
            correct = correct*0;
        end
    end

    if(correct == 1)
        theta_one_sol = [theta_one_sol; theta_output(i,:)];
    end

end
%check angels
end