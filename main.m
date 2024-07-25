ccc
Init_params
%% make spline reference path
ax = [0.0, 10.0, 6.0, 20.0, 44.0];
ay = [0.0, 20.0, 20.0, 35.0, 15.0];
[cx, cy, cyaw, ck, s     ] = calc_spline_course(ax, ay, dl);
ax = [44.0, 15.0, 0.0, 0.0];
ay = [15.0, 22.0, 5.0, 0.0];
[cx2, cy2, cyaw2, ck2, s2] = calc_spline_course(ax, ay, dl);
cyaw2 = cyaw2 - pi;
cx = [cx; cx2];
cy = [cy; cy2];
cyaw = [cyaw; cyaw2];
ck = [ck; ck2];
cyaw = smooth_yaw(cyaw);
cz = zeros(length(cx), 1); % only for visualization in 3D
%% calculate speed profile with reference path
sp = calc_speed_profile(cx, cy, cyaw, target_speed);
initial_state = State(cx(1), cy(1), cyaw(1), 0.0);
%% do simulation
mpc = MPC(4, 2, 5, animate);
path_len = length(cx);
goal = [cx(end), cy(end)];
state = initial_state;
state.yaw = wrapToPi(state.yaw - cyaw(1)) + cyaw(1);

[x, y, yaw, v] = deal(state.x, state.y, state.yaw, state.v);
[t, d, a] = deal(0);
odelta = NaN;
oa = NaN;

[target_ind, ~] = mpc.calc_nearest_index(state, cx, cy, cyaw, 1);
while max_time >=time
    [xref, target_ind, dref] = mpc.calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind);
    x0 = [state.x, state.y, state.v, state.yaw];
    [oa, odelta, ox, oy, oyaw, ov] = mpc.iterative_linear_mpc_control(xref, x0, dref, oa, odelta);
    if ~isnan(odelta)
        di = odelta(1);
        ai = oa(1);
        state = mpc.update_state(state, ai, di);
    end       

    time = time + mpc.dt;
    x(end+1) = state.x;
    y(end+1) = state.y;
    yaw(end+1) = state.yaw;
    v(end+1) = state.v;
    t(end+1) = time;
    d(end+1) = di;
    a(end+1) = ai;

    if animate == true
        oz = zeros(length(ox), 1);
        mpc.bodyObj.pos_com = [state.x;state.y;0];
        mpc.bodyObj.update();
        fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,'view_info',[60,45],'axis_info',[-5, 50, -5, 50, -1, 2],'SET_DRAGZOOM',1,'GRID_ON',1);
        plot_cube(mpc.bodyObj.T_val, [-mpc.bodyObj.size(1)/2, -mpc.bodyObj.size(2)/2, -mpc.bodyObj.size(3)/2], mpc.bodyObj.size, ...
        'subfig_idx',2,'color','r','alpha',1,'edge_color','r'); % plot cube 
        plot_curve_3d(cx,cy,cz,'subfig_idx', 1, 'color', 'k');
        path_color = [0.2, 0.2, 0.2];
        for i=1:length(ox)
            path = [ox(i), oy(i), oz(i)];
            plot_spheres(path, 'subfig_idx', i, 'sr', 0.1,'colors',path_color, 'sfa',0.8);
            % plot_spheres(path(:, i)','subfig_idx',i,...
            % 'sr', 0.001,'colors',path_color,'sfa',0.8);
        end
        drawnow;
        if ~ishandle(fig), break; end
    end 
    if mpc.check_goal(state, goal, target_ind, length(cx))
        disp("goal")
        break
    end 
end
%% plot splie course
figure()
plot(cx, cy, 'k.-')
hold on
plot(x, y, 'r.-')
grid


