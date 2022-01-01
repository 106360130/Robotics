function theta = kinematics_inverse_2(T)
    a = [0.12, 0.25, 0.26, 0, 0, 0];
    theta = zeros(6, 1);
    
    X_c = T(1, 4);
    Y_c = T(2, 4);
    Z_c = T(3, 4);

    %find theta1
    theta(1) = atan2(Y_c, X_c);

    %find theta3
    t = (X_c^2 + Y_c^2)^0.5;
    r = t - a(1);
    s = abs(Z_c);
    D = (s^2 + r^2 - a(2)^2 - a(3)^2) / (2*a(2)*a(3));
    theta(3) = atan2(-(1-D^2)^0.5, D);


    %find theta2
    %{
    temp_1 = atan2(Z_c, r);  %Alpha
    temp_2 = atan2(a(3)*sin(theta(3)), a(2) + a(3)*cos(theta(3)));  %Beta
    theta(2) = temp_1 - temp_2;
    %} 
    px = X_c;
    py = Y_c;
    pz = Z_c;
    C = cos(theta(1))*px + sin(theta(1))*py;
    M = 0.26^2 - C^2 -pz^2 - 0.25^2 -0.12^2 +0.24*C;
    A = 0.5 * pz;
    B = 0.06 - 0.5*C;
    try
        theta_2_1 = atan2(M/((A^2+B^2)^0.5), (1-M^2/(A^2+B^2))^0.5) - atan2(B, A);
        theta_2_2 = atan2(M/((A^2+B^2)^0.5), -(1-M^2/(A^2+B^2))^0.5) - atan2(B, A);
    catch
        theta_2_1 = 10000;
        theta_2_2 = 10000;
    end
    
    if(-30 <= theta_2_1*(180/pi) && theta_2_1*(180/pi) <= 100)
        theta(2) = theta_2_1;
    
    else
        theta(2) = theta_2_2;

    end

    

    %find theta5
    r13 = T(1, 3);
    r23 = T(2, 3);
    S1 = sin(theta(1));
    C1 = cos(theta(1));
    d = -r13*S1 + r23*C1;  %正負更改
    %d = abs(d);
    theta(5) = atan2((1-(d^2))^0.5, d);

    

    %find theta4
    C23 = cos(theta(2)+theta(3));
    S23 = sin(theta(2)+theta(3));
    r33 = T(3, 3);
    temp_1 = C1*C23*r13 + S1*C23*r23 - S23*r33;  %S5C4
    temp_2 = -C1*S23*r13 - S1*S23*r23 - C23*r33;  %S4S5
    theta(4) = atan2(temp_2, temp_1);


    
    %find theta6
    r11 = T(1, 1);
    r21 = T(2, 1);
    r12 = T(1, 2);
    r22 = T(2, 2);
    %temp_1 = -S1*r11 + C1*r21;  %-S5C6
    temp_1 = -S1*r11 + C1*r21;  %S5C6
    temp_2 = S1*r12 - C1*r22;  %S5S6
    theta(6) = atan2(temp_2, temp_1);


end
