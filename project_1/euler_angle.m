function euler_angles = euler_angle(T)
    r33 = T(3, 3);
    r13 = T(1, 3);
    r23 = T(2, 3);
    r31 = T(3, 1);
    r32 = T(3, 2);
    

    euler_angles = zeros(3, 1);
    euler_angles(2) = atan2((1-r33^2)^0.5, r33);  %theta
    euler_angles(1) = atan2(r23, r13);  %phi
    euler_angles(3) = atan2(r32, -r31);  %psi

    euler_angles = euler_angles .* (180/pi);  %徑度換角度

end