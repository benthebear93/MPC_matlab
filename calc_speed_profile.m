function speed_profile = calc_speed_profile(cx, cy, cyaw, target_speed)

    speed_profile = repmat(target_speed, 1, length(cx));
    direction = 1.0;  % forward

    % Set stop point
    for i = 1:length(cx) - 1
        dx = cx(i + 1) - cx(i);
        dy = cy(i + 1) - cy(i);

        move_direction = atan2(dy, dx);

        if dx ~= 0.0 && dy ~= 0.0
            dangle = abs(pi_2_pi(move_direction - cyaw(i)));
            if dangle >= pi / 4.0
                direction = -1.0;
            else
                direction = 1.0;
            end
        end

        if direction ~= 1.0
            speed_profile(i) = - target_speed;
        else
            speed_profile(i) = target_speed;
        end
    end

    speed_profile(end) = 0.0;

end