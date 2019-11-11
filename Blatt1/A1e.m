clear();
covariance = [100, 0; 0, 10];
mu = [0; 0];
samples = randomSamples(mu, covariance, 100000);
scatter(samples(1, :), samples(2, :));

covv = cov(samples');
mean = mean(samples');
maxScale = max(covariance(1, :), covariance(2, :));
x1 = -maxScale:1:maxScale;
x2 = -maxScale:1:maxScale;
[X1,X2] = meshgrid(x1,x2);
X = [X1(:) X2(:)];
y = mvnpdf(X, mean, covv);
y = reshape(y,length(x2),length(x1));
figure();
surf(x1, x2, y);
xlabel("x");
ylabel("y");

