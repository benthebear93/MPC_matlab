classdef State<handle
    % robot states

    properties (Access = public)
        x
        y
        yaw
        v
        predelta
    end
    methods
        function obj = State(x, y, yaw, v)
            obj.x = x;
            obj.y = y;
            obj.yaw = yaw;
            obj.v = v;
        end
    end
end
