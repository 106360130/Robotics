function T_result = T_individual(theta, d, a, alpha)
    %單一個的轉換矩陣
    T_result = [cos_r(theta), -sin_r(theta)*cos_r(alpha), sin_r(theta)*sin_r(alpha), a*cos_r(theta);
        sin_r(theta), cos_r(theta)*cos_r(alpha), -cos_r(theta)*sin_r(alpha), a*sin_r(theta);
        0, sin_r(alpha), cos_r(alpha), d;
        0, 0, 0, 1];

end