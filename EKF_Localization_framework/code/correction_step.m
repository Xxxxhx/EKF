function [mu, sigma] = correction_step(mu, sigma, z, l)
    % Updates the belief, i. e., mu and sigma, according to the sensor model
    %
    % The employed sensor model is range-only.
    %
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution
    % z: structure containing the landmark observations, see
    %    read_data for the format
    % l: structure containing the landmark position and ids, see
    %    read_world for the format

    % current state
    x = mu(1);
    y = mu(2);
    t = mu(3);
    
    % Compute the expected range measurements.
    % This corresponds to the function h.
    expected_ranges = zeros(size(z, 2), 1);
    for i = 1:size(z, 2)
	lx = l(z(i).id).x;
	ly = l(z(i).id).y;
        expected_ranges(i) = sqrt((lx-x)*(lx-x) + (ly-y)*(ly-y)); 
    end

    % Jacobian of h
    H = zeros(size(z, 2), 3);

    % Measurements in vectorized form
    Z = zeros(size(z, 2), 1);
    for i = 1:size(z, 2)
	lx = l(z(i).id).x;
	ly = l(z(i).id).y;
        H(i, :) = [ (x-lx) / sqrt((lx-x)*(lx-x) + (ly-y)*(ly-y));
		  (y-ly) / sqrt((lx-x)*(lx-x) + (ly-y)*(ly-y));
		  0 ];
        Z(i) = z(i).range;
    end

    R = diag(repmat([0.5], size(z, 2), 1));

    K = sigma * H' * (H * sigma * H' + R)^(-1)
    mu = mu + K * (Z - expected_ranges)
    sigma = (eye(3) - K * H) * sigma
end