function q = portion_accelerated(time_interval, sample_rate, T, t_acc, noap_A, noap_B, noap_C)

    delta_B = noap_A - noap_B;
    delta_C = noap_C - noap_B;

    i = 1;
    for sample_time = (time_interval(1) + sample_rate) : sample_rate : (time_interval(2) - sample_rate)

        temp_t = time_interval(2) - time_interval(1);
        h = (sample_time + t_acc)/temp_t;
        temp_1 = (delta_C*(t_acc/T) + delta_B);
    
        q(:, :, i) = (temp_1*(2-h)*h^2 - 2*delta_B)*h + noap_B + delta_B;  
    
        i = i+1;
    
    end

end

