function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)


% TODO: Compute the 3x3 Jacobian Gx of the motion model
theta = mu(3);
N = (size(sigma,1) - 3) / 2;
F_x = [eye(3) zeros(3, 2 * N)];
Gx = eye(3) + [zeros(3,2) [-u.t * sin(theta + u.r1); u.t * cos(theta + u.r1); 0]];

% TODO: Construct the full Jacobian G
G = eye(size(F_x, 2)) + F_x' * Gx * F_x;

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% Compute the predicted sigma after incorporating the motion
mu = mu + [eye(3) zeros(3, 2 * N)]' * [u.t * cos(theta + u.r1); u.t * sin(theta + u.r1); u.r1 + u.r2];
mu(3) = normalize_angle(mu(3));
sigma = G * sigma * G' + R;
end