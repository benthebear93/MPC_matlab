classdef CubicSpline2D<handle
    properties
       s
       sx
       sy
    end

    methods
        function obj = CubicSpline2D(x, y)
            obj.s = obj.calc_s(x, y);
            obj.sx = CubicSpline1D(obj.s, x);
            obj.sy = CubicSpline1D(obj.s, y);
        end

        function s = calc_s(obj, x, y)
            dx = diff(x);
            dy = diff(y);
            ds = hypot(dx, dy);
            s = [0, cumsum(ds)];
        end

        function [x, y] = calc_position(obj, s)
            x = obj.sx.calc_position(s);
            y = obj.sy.calc_position(s);
        end

        function k = calc_curvature(obj, s)
            dx = obj.sx.calc_first_derivative(s);
            ddx = obj.sx.calc_second_derivative(s);
            dy = obj.sy.calc_first_derivative(s);
            ddy = obj.sy.calc_second_derivative(s);
            k = (ddy .* dx - ddx .* dy) ./ ((dx .^ 2 + dy .^ 2).^(3 / 2));
        end
        function yaw = calc_yaw(obj, s)
            dx = obj.sx.calc_first_derivative(s);
            dy = obj.sy.calc_first_derivative(s);
            yaw = atan2(dy, dx);
        end

    end

end
