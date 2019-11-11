clear;
data = load("data.mat");
data = data.data;
mean = mean(data')
covariance = cov(data')
figure();
scatter(data(1, :), data(2, :));
hold on;
x1 = -5:.2:10;
x2 = -5:.2:10;
[X1,X2] = meshgrid(x1,x2);
X = [X1(:) X2(:)];
y = mvnpdf(X, mean, covariance);
y = reshape(y,length(x2),length(x1));
contour(x1, x2, y, [0.0001, 0.001, 0.01, 0.05, 0.15, 0.25, 0.35]);
xlabel('x');
ylabel('y');