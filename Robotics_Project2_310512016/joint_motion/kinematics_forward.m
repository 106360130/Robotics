function T = kinematics_forward(theta)
    
    %已知角度求位置
    a = [0.12, 0.25, 0.26, 0, 0, 0];
    d = [0, 0, 0, 0, 0, 0];
    alpha=[-90, 0, 0, -90, 90, 0];
    
    T = T_individual(theta(1), d(1), a(1), alpha(1));
    for i = 2:6
        T = T * T_individual(theta(i), d(i), a(i), alpha(i));
    end

end