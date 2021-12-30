function q = accelerated_portion(A, B, C, T, t, t_acc)
T = 2*t_acc;
%accelerated_portion
delta_C = C - B;
delta_B = A - B;

h = (t + t_acc)/(2*t_acc);

temp_1 = delta_C.*t_acc.*(1./T) + delta_B;  %4*4
temp_2 = (2-h).*(h.^2);  %1*501
temp_3 = 2.*delta_B;  %4*4

q = zeros(4, 4, length(t));

for i = 1:length(t)
    q(:, :, i) = (temp_1*temp_2(i) - temp_3).*h(i) + B + delta_B;
end
%accelerated_portion

end