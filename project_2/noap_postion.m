function position = noap_postion(noap)

size_noap = size(noap);
q_x = zeros(size_noap(3), 1);
q_y = zeros(size_noap(3), 1);
q_z = zeros(size_noap(3), 1);
for i = 1:size_noap(3)
    q_x(i) = noap(1, 4, i)/100;
    q_y(i) = noap(2, 4, i)/100;
    q_z(i) = noap(3, 4, i)/100;
end

position = [q_x q_y q_z];

end