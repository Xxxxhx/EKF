% This is the main extended Kalman filter. This script calls all the required
% functions in the correct order.
%
% You can disable the plotting or change the number of steps the filter
% runs for to ease the debugging. You should however not change the order
% or calls of any of the other lines, as it might break the framework.
%
% If you are unsure about the input and return values of functions you
% should read their documentation which tells you the expected dimensions.

% Make librobotics available
addpath('librobotics');

% Read world data, i.e. landmarks
disp("Reading world data ...");
landmarks = read_world('../data/world.dat');
disp(" done\n");
% Read sensor readings, i.e. odometry and range-bearing sensor
disp("Reading sensor data ...");
data = read_data('../data/sensor_data.dat');
disp(" done\n");

% Initialize belief
mu = [0.0; 0.0; 0.0];
sigma = [1.0, 0.0, 0.0; 
    0.0, 1.0, 0.0; 
    0.0, 0.0, 1.0];
    
% Perform filter update for each odometry, observation pair read from the
% data file.
for t = 1:size(data.timestep, 2)

    % Perform the prediction step of the EKF
    [mu, sigma] = prediction_step(mu, sigma, data.timestep(t).odometry);

    % Perform the correction step of the EKF
    [mu, sigma] = correction_step(mu, sigma, data.timestep(t).sensor, landmarks);

    % Generate visualization plots of the current state of the filter
    plot_state(mu, sigma, landmarks, t);
end

% Display the final state estimate
disp("Final pose: ")
disp("mu = "), disp(mu), disp(", sigma = "), disp(sigma);
