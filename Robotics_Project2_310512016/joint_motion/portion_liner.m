function [q dq ddq] = portion_liner(time_interval, sample_rate, T, delta, theta_B)

    i=1;
    for sample_time = time_interval(1) : sample_rate : time_interval(2)
        h = sample_time/T;
        q(:, i) = delta*h + theta_B;
        dq(:, i) = delta/T;
        ddq(:, i) = [0; 0; 0; 0; 0; 0];
        
        i = i+1;

    end

end