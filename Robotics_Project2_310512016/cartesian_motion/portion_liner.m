function q = portion_liner(time_interval, sample_rate, T, delta, noap_B)

    i=1;
    for sample_time = time_interval(1) : sample_rate : time_interval(2)
        h = sample_time/T;
        q(:, :, i) = delta*h + noap_B;
        
        i = i+1;

    end

end

