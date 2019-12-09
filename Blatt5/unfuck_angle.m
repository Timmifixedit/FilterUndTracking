function ret = unfuck_angle(x, ind)
    ret = x;
    if ret(ind) < -pi
        ret(ind) = 2 * pi + ret(ind);
    end
    if ret(ind) > pi
        ret(ind) = ret(ind) - 2 * pi;
    end
end