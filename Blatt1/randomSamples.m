function [ ret ] = randomSamples(mean, covariance, samples)
    dim = size(covariance, 1);
    rndVec = randn(dim, samples);
    [V, D] = eig(covariance);
    VInv = inv(V');
    ret = VInv * sqrt(D) * rndVec + mean;
end

