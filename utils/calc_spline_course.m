function [rx, ry, ryaw, rk, s] = calc_spline_course(x, y, ds)
    if nargin < 3
        ds = 0.1;  % 기본 스텝 사이즈 설정
    end

    % CubicSpline2D 클래스의 인스턴스 생성
    sp = CubicSpline2D(x, y);
    s = 0:ds:sp.s(end);
    % 결과를 저장할 배열 초기화
    rx = zeros(size(s,2), 1);
    ry = zeros(size(s,2), 1);
    ryaw = zeros(size(s,2), 1);
    rk = zeros(size(s,2), 1);

    % 각 s에 대해 위치, yaw, 곡률 계산
    for i = 1:length(s)
        [rx(i), ry(i)] = sp.calc_position(s(i));
        ryaw(i) = sp.calc_yaw(s(i));
        rk(i) = sp.calc_curvature(s(i));
    end
end