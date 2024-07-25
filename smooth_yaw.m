function yaw = smooth_yaw(yaw)

    for i = 1:length(yaw) - 1
        dyaw = yaw(i + 1) - yaw(i);

        while dyaw >= pi / 2.0
            yaw(i + 1) = yaw(i + 1) - pi * 2.0;
            dyaw = yaw(i + 1) - yaw(i);
        end

        while dyaw <= -pi / 2.0
            yaw(i + 1) = yaw(i + 1) + pi * 2.0;
            dyaw = yaw(i + 1) - yaw(i);
        end
    end
end