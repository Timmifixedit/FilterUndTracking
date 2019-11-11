clear;
signals = load("samples.mat");
signals = signals.data;

A = sym('a', [1, 4]);
p = sym('p', [1, 100]);
for i = 1 : 100
    p(i) = A * [1, i, i^2, i^3]';
end

figure();
for i = 1:200
    signal = signals(i, :);
    sym error;
    error = sum((p - signal).^2);
    derivative = gradient(error, A);
    eq = derivative == [0, 0, 0, 0]';
    res = solve(eq, A);
    x = 1 : 100;
    y = res.a1 + res.a2 * x + res.a3 * x.^2 + res.a4 * x.^3;
    plot(x, y);
    hold on;
    plot(1:100, signal);
    hold off;
    pause(0.1);
end
