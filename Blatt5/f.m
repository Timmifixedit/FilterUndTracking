function [ x_new ] = f( x, T )
    v = x(3);
    psi = x(4);
    omega = x(5);
    x_new = x + [v * T * cos(psi), v * T * sin(psi), 0, T * omega, 0]';
end