% Cubic splie examples 

%% Cubic spline 1d
ccc
x = [1, 2, 3, 4, 5];
y = [1.7, -6, 5, 6.5, 0.0];

sp = CubicSpline1D(x, y);

xi = linspace(1, 5);
yi = zeros(1, 100);
for i=1:99
    yi(i) = sp.calc_position(xi(i));
end

plot(x, y, 'xb', 'DisplayName', 'Data points');
hold on;
plot(xi, yi, 'r', 'DisplayName', 'Cubic spline interpolation');
grid on;
legend;
hold off;
%% Cubic splie 2d
ccc
x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0];
ds = 0.1;  % [m] distance of each interpolated points
sp = CubicSpline2D(x, y);
s = 0:ds:sp.s(end);

rx = [];
ry = [];
ryaw = [];
rk = [];
for i_s = s
    [ix, iy] = sp.calc_position(i_s);
    rx = [rx, ix];
    ry = [ry, iy];
    ryaw = [ryaw, sp.calc_yaw(i_s)];
    rk = [rk, sp.calc_curvature(i_s)];
end

% Cubic spline path
figure;
plot(x, y, 'xb', 'DisplayName', 'Data points');
hold on;
plot(rx, ry, '-r', 'DisplayName', 'Cubic spline path');
grid on;
axis equal;
xlabel('x[m]');
ylabel('y[m]');
legend;
hold off;

% Yaw angle
figure;
plot(s, rad2deg(ryaw), '-r', 'DisplayName', 'yaw');
grid on;
legend;
xlabel('line length[m]');
ylabel('yaw angle[deg]');

% Curvature
figure;
plot(s, rk, '-r', 'DisplayName', 'curvature');
grid on;
legend;
xlabel('line length[m]');
ylabel('curvature [1/m]');
