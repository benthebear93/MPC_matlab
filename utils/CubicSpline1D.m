classdef CubicSpline1D
    properties
        x
        y
        a
        b
        c
        d
        nx
    end
    
    methods
        function obj = CubicSpline1D(x, y)
            h = diff(x);
            if any(h<0)
                error("x coordinates must be sorted in ascending order");
            end
            obj.x = x;
            obj.y = y;
            nx = size(x);
            obj.nx = nx(2);
            obj.a = y;

            % calc coeff c
            A = obj.calc_A(h);
            B = obj.calc_B(h, obj.a);
            obj.c = A \ B;

            % calc spline coeff b and d
            obj.b = zeros(1, obj.nx-1);
            obj.d = zeros(1, obj.nx-1);
            for i = 1:obj.nx-1
                obj.d(i) = (obj.c(i+1) - obj.c(i)) / (3.0 * h(i));
                obj.b(i) = (obj.a(i+1) - obj.a(i)) / h(i) - h(i) * (2.0 * obj.c(i) + obj.c(i+1)) / 3.0;
            end
        end
      function position = calc_position(obj, x)
            if x < obj.x(1) || x > obj.x(end)
                position = NaN;
                return;
            end
            
            i = obj.search_index(x);
            dx = x - obj.x(i);
            position = obj.a(i) + obj.b(i) * dx + obj.c(i) * dx^2 + obj.d(i) * dx^3;
        end
        
        function dy = calc_first_derivative(obj, x)
            if x < obj.x(1) || x > obj.x(end)
                dy = NaN;
                return;
            end
            
            i = obj.search_index(x);
            dx = x - obj.x(i);
            dy = obj.b(i) + 2.0 * obj.c(i) * dx + 3.0 * obj.d(i) * dx^2;
        end
        
        function ddy = calc_second_derivative(obj, x)
            if x < obj.x(1) || x > obj.x(end)
                ddy = NaN;
                return;
            end
            
            i = obj.search_index(x);
            dx = x - obj.x(i);
            ddy = 2.0 * obj.c(i) + 6.0 * obj.d(i) * dx;
        end
    end
    
    methods (Access = private)
        function i = search_index(obj, x)
            i = find(obj.x <= x, 1, 'last');
        end
        
        function A = calc_A(obj, h)
            A = zeros(obj.nx);
            A(1, 1) = 1.0;
            for i = 1:obj.nx-1
                if i ~= obj.nx-1
                    A(i+1, i+1) = 2.0 * (h(i) + h(i+1));
                end
                A(i+1, i) = h(i);
                A(i, i+1) = h(i);
            end
            A(1, 2) = 0.0;
            A(end, end-1) = 0.0;
            A(end, end) = 1.0;
        end
        
        function B = calc_B(obj, h, a)
            B = zeros(obj.nx, 1);
            for i = 1:obj.nx-2
                B(i+1) = 3.0 * (a(i+2) - a(i+1)) / h(i+1) - 3.0 * (a(i+1) - a(i)) / h(i);
            end
        end
    end
end
