function [mu, sigma] = prediction_step(mu, sigma, u)
    % Updates the belief, i. e., mu and sigma, according to the motion model
    %
    % u: odometry reading (r1, t, r2)
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution

    % current state
    x = mu(1);
    y = mu(2);
    t = mu(3);
    
    % current odometry
    r1 = u.r1;
    trans = u.t;
    r2 = u.r2;
    
    % Compute the noise-free motion. This corresponds to the function g, evaluated
    % at the state mu.

    mu = [ x + trans * cos(t + r1);
	  y + trans * sin(t + r1);
	  t + r1 + r2];
	  
    % Compute the Jacobian of g with respect to the state
    G = [1, 0, -sin(t + r1)*trans;
	0, 1, cos(t + r1)*trans;
	0, 0, 1];

    % Motion noise
    Q = [0.2, 0, 0; 
        0, 0.2, 0; 
        0, 0, 0.02];

    sigma = G * sigma * G' + Q;
end