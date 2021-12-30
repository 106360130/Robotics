function q = linear_portion(B, C, T, t)

%linear_portion
delta_C = C - B;

h = t/T;

q = zeros(4, 4, length(t));
for i = 1:length(h)
    q(:, :, i) = delta_C.*h(i) + B;
end
%linear_portion

end