function angle = pi_2_pi(angle)
    while angle > pi
        angle = angle - 2*pi;
    end

    while angle < -pi
        angle = angle + 2.0*pi;
    end
end
