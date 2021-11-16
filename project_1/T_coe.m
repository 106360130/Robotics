function T_result = T_coe(d, a, alpha)
    %計算theta之外的係數
    T_result = [1, -1*cos_r(alpha), 1*sin_r(alpha), a*1;
        1, 1*cos_r(alpha), -1*sin_r(alpha), a*1;
        0, sin_r(alpha), cos_r(alpha), d;
        0, 0, 0, 1];

end