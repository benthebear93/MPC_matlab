classdef MPC<handle
    properties (Access = public)
        NX = 4; % x, y, v, yaw
        NU = 2; % acc, steer
        T = 5; % Horizion Lenght
        
        R = diag([0.01, 0.01]);
        R_col = diag([0.01, 0.01]);
        Rd = diag([0.01, 1.0]);
        Q = diag([1.0, 1.0, 0.5, 0.5]);
        Qf =  diag([1.0, 1.0, 0.5, 0.5]);

        goal_dis = 2;
        stop_speed = 0.5 / 3.6;
        
        max_iter = 3;
        du_th = 0.1;
        
        target_speed = 10 / 3.6;
        n_ind_search = 10; % search index number
        
        dt = 0.2;
        
        lenght = 4.5;
        widht = 2.0;
        backtowheel = 1.0;
        wheel_len = 0.3;
        wheel_widht = 0.2;
        tread = 0.7;
        wb = 2.5;
        
        max_steer = 45*0.0175;
        max_dsteer = 30*0.0175;
        max_speed = 55/3.6;
        min_speed = -20/3.6;
        max_acc = 1.0;
        bodyObj
    end
    methods
        function obj = MPC(NX, NU, T, animate)
            obj.NX = NX;
            obj.NU = NU;
            obj.T = T;
            for i=1:obj.T-1
                obj.R_col=blkdiag(obj.R_col,obj.R);
            end
            disp("Init Model predictive controller");
            if animate == true
                obj.bodyObj = body('car', 10, eye(3), [0; 0; 0], [0; 0; 0], [1; 1; 1], 0.1, 0.1, false, 'cube');
            end
        end
        function [A, B, C] = get_linear_model_matrix(obj, v, phi, delta)
            A = zeros(obj.NX, obj.NX);
            A(1,1) = 1;
            A(2,2) = 1;
            A(3,3) = 1;
            A(4,4) = 1;
            A(1,3) = obj.dt * cos(phi);
            A(1,4) = -obj.dt * v * sin(phi);
            A(2,3) = obj.dt * sin(phi);
            A(2,4) = obj.dt * v * cos(phi);
            A(4,3) = obj.dt * tan(delta) / obj.wb;

            B = zeros(obj.NX, obj.NU);
            B(3, 1) = obj.dt;
            B(4, 2) = obj.dt * v/ (obj.wb * cos(delta)^2);
            
            C = zeros(obj.NX, 1);
            C(1) = obj.dt *v *sin(phi)*phi;
            C(2) = -obj.dt * v *cos(phi) * phi; 
            C(4) = -obj.dt * v * delta / (obj.wb * cos(delta)^2);
        end
        
        function state = update_state(obj, state, a, delta)

            if delta >= obj.max_steer
                delta = obj.max_steer;
            elseif delta <= -obj.max_steer
                delta = -obj.max_steer;
            end
            state.x = state.x + state.v * cos(state.yaw)*obj.dt;
            state.y = state.y + state.v * sin(state.yaw)*obj.dt;
            state.yaw = state.yaw + state.v / obj.wb * tan(delta)*obj.dt;
            state.v = state.v + a*obj.dt;

            if state.v > obj.max_speed
                state.v = obj.max_speed;
            elseif state.v < obj.min_speed
                state.v = obj.min_speed;
            end
        end

        function [ind, mind] = calc_nearest_index(obj, state, cx, cy, cyaw, pind)
            idx_last = pind+ obj.n_ind_search;
            if idx_last > length(cx)
                idx_last = length(cx);
            end
            dx = state.x - cx(pind:(idx_last- 1));
            dy = state.y - cy(pind:(idx_last- 1));

            d = dx.^2 + dy.^2;
            mind = min(d);
            ind = find(d==mind, 1) + pind - 1;
            mind = sqrt(mind);
            dxl = cx(ind) - state.x;
            dyl = cy(ind) - state.y;
            angle = pi_2_pi(cyaw(ind) - atan2(dyl, dxl));
            if angle < 0
                mind = mind*-1;
            end
        end

        function xbar = predict_motion(obj, x0, oa, od, xref)
            xbar = zeros(size(xref));
            xbar(:, 1) = x0;

            state = State(x=x0(1), y=x0(2), yaw=x0(4), v=x0(3));
            for i = 1:obj.T
                ai = oa(i);
                di = od(i);
                state = update_state(state, ai, di);
                xbar(1, i + 1) = state.x;
                xbar(2, i + 1) = state.y;
                xbar(3, i + 1) = state.v;
                xbar(4, i + 1) = state.yaw;
            end
        end
    
        function [oa, odelta, ox, oy, oyaw, ov] = linear_mpc_control(obj, xref, xbar, x0, dref)
            %initial guess for optimization variables
            x0_vars = zeros(obj.NX*(obj.T+1) + obj.NU*obj.T, 1);
        
            % cost function
            cost_function = @(vars) cost_fun(obj, vars, xref);
            % constraints
            constraint_function = @(vars) constraint_fun(obj, vars, xbar, dref, x0);
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = [];
            ub = [];
            % optimization options
            options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');
            [vars_opt, ~, exitflag] = fmincon(cost_function, x0_vars, A, b, Aeq, beq, lb, ub, constraint_function, options);

            if exitflag > 0
                ox = vars_opt(1:obj.NX:end);
                oy = vars_opt(2:obj.NX:end);
                ov = vars_opt(3:obj.NX:end);
                oyaw = vars_opt(4:obj.NX:end);
                oa = vars_opt(obj.NX*(obj.T+1)+1:obj.NU:end);
                odelta = vars_opt(obj.NX*(obj.T+1)+2:obj.NU:end);
            else
                disp('Error: Cannot solve mpc..');
                oa = NaN;
                odelta = NaN;
                ox = NaN;
                oy = NaN;
                oyaw = NaN;
                ov = NaN;
            end
        end
        function cost = cost_fun(obj, vars, xref)
            % Extract variables
            x = reshape(vars(1:obj.NX*(obj.T+1)), obj.NX, obj.T+1);
            u = reshape(vars(obj.NX*(obj.T+1)+1:end), obj.NU, obj.T);

            cost = 0;
            for t = 1:obj.T
                cost = cost + u(:, t)' * obj.R * u(:, t);
                if t ~= 1
                    cost = cost + (xref(:, t) - x(:, t))' * obj.Q * (xref(:, t) - x(:, t));
                end
                if t < obj.T
                    cost = cost + (u(:, t+1) - u(:, t))' * obj.Rd * (u(:, t+1) - u(:, t));
                end
            end
            cost = cost + (xref(:, obj.T+1) - x(:, obj.T+1))' * obj.Qf * (xref(:, obj.T+1) - x(:, obj.T+1));
        end
        
        function [c, ceq] = constraint_fun(obj, vars, xbar, dref, x0)
            %% Extract variables
            x = reshape(vars(1:obj.NX*(obj.T+1)), obj.NX, obj.T+1);
            u = reshape(vars(obj.NX*(obj.T+1)+1:end), obj.NU, obj.T);
            %% Initialize constraints
            ceq = []; % eqaulity constraints
            c = []; % inequality constratins

            % Initial state constraint
            ceq = [ceq; x(:, 1) - x0'];
            for t = 1:obj.T
                [A, B, C] = get_linear_model_matrix(obj, xbar(3, t), xbar(4, t), dref(1, t));
                ceq = [ceq; x(:, t+1) - (A * x(:, t) + B * u(:, t) + C)]; % getting each step states check
                if t < obj.T
                    c = [c; abs(u(2, t+1) - u(2, t)) - obj.max_dsteer * obj.dt];
                end
            end
            % Speed constraints
            c = [c; x(3, :)' - obj.max_speed]; % getting only the speed value from states along time horizion v_t1, v_t2, v_t3, v_t4, v_t5)
            c = [c; obj.min_speed - x(3, :)'];
            % Control constraints
            c = [c; abs(u(1, :))' - obj.max_acc];
            c = [c; abs(u(2, :))' - obj.max_steer];
        end
       function is_goal_reached = check_goal(obj, state, goal, tind, nind)
            dx = state.x - goal(1);
            dy = state.y - goal(2);
            d = hypot(dx, dy);
        
            isgoal = (d <= obj.goal_dis);
        
            if abs(tind - nind) >= 5
                isgoal = false;
            end
        
            isstop = (abs(state.v) <= obj.stop_speed);
        
            if isgoal && isstop
                is_goal_reached = true;
            else
                is_goal_reached = false;
            end
        
        end
        function xbar = prediction(obj, x0, oa, od, xref)
            xbar = xref * 0;
            xbar(:, 1) = x0;
        
            state.x = x0(1);
            state.y = x0(2);
            state.v = x0(3);
            state.yaw = x0(4);
            
            for i=1:obj.T
                ai = oa(i);
                di = od(i);
                state = update_state(obj, state, ai, di);
                xbar(1, i+1) = state.x;
                xbar(2, i+1) = state.y;
                xbar(3, i+1) = state.v;
                xbar(4, i+1) = state.yaw;
            end
        end

        function [oa, od, ox, oy, oyaw, ov] = iterative_linear_mpc_control(obj, xref, x0, dref, oa, od)
            ox = [];
            oy = [];
            oyaw = [];
            ov = [];

            if any(isnan(oa)) || any(isnan(od))
                oa = zeros(1, obj.T);
                od = zeros(1, obj.T);
            end
            for i=1:obj.max_iter
                xbar = prediction(obj, x0, oa, od, xref);

                poa = oa; pod = od;
                [oa, od, ox, oy, oyaw, ov] = linear_mpc_control(obj, xref, xbar, x0, dref);
                du = sum(abs(oa-poa)) + sum(abs(od-pod));
                if du <= obj.du_th
                    break
                end
            end
        end

        function [xref, ind, dref] = calc_ref_trajectory(obj, state, cx, cy, cyaw, ~, sp, dl, pind)
            xref = zeros(obj.NX, obj.T + 1);
            dref = zeros(1, obj.T + 1);
            ncourse = length(cx);
            [ind, ~] = calc_nearest_index(obj, state, cx, cy, cyaw, pind);
            if pind >= ind
                ind = pind;
            end
        
            xref(1, 1) = cx(ind);
            xref(2, 1) = cy(ind);
            xref(3, 1) = sp(ind);
            xref(4, 1) = cyaw(ind);
            dref(1, 1) = 0.0; 
        
            travel = 0.0;
        
            for i = 2:obj.T + 1
                travel = travel + abs(state.v) * obj.dt;
                dind = round(travel / dl);
        
                if (ind + dind) < ncourse
                    xref(1, i) = cx(ind + dind);
                    xref(2, i) = cy(ind + dind);
                    xref(3, i) = sp(ind + dind);
                    xref(4, i) = cyaw(ind + dind);
                    dref(1, i) = 0.0;
                else
                    xref(1, i) = cx(ncourse);
                    xref(2, i) = cy(ncourse);
                    xref(3, i) = sp(ncourse);
                    xref(4, i) = cyaw(ncourse);
                    dref(1, i) = 0.0;
                end
            end
        end
    end
end
