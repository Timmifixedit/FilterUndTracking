function [derivative] = f_x(x, T)
    v = x(3);
    psi = x(4);
    derivative = eye(5) + [0, 0, 0, 0, 0;
                           0, 0, 0, 0, 0;
                           T * cos(psi), T * sin(psi), 0, 0, 0;
                           -v * T * sin(psi), v * T * cos(psi), 0, 0, 0;
                           0, 0, 0, T, 0]';
end