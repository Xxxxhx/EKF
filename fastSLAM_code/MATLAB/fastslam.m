% This is the main FastSLAM loop. This script calls all the required
% functions in the correct order.
%
% You can disable the plotting or change the number of steps the filter
% runs for to ease the debugging. You should however not change the order
% or calls of any of the other lines, as it might break the framework.
%
% If you are unsure about the input and return values of functions you
% should read their documentation which tells you the expected dimensions.

clear all;
close all;

% Make tools available
addpath('tools');

% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks = read_world('../data/world.dat');
% Read sensor readings, i.e. odometry and range-bearing sensor
data = read_data('../data/sensor_data.dat');

% Get the number of landmarks in the map
NUM_LANDMARKS = size(landmarks,2);

% Set the noise of motion model (odemetry): [alpha1, alpha2, alpha3, alpha4]
MOTION_NOISE = [0.1, 0.1, 0.05, 0.05]';

% Set the noise of motion model: Q_t
SENSOR_NOISE = [1.0, 0; ...
                0,   0.1]';

% how many particles
NUM_PARTICLES = 100;

% initialize the particles array
for i = 1:NUM_PARTICLES
  particles(i).weight = 1 / NUM_PARTICLES;
  particles(i).pose = zeros(3,1);
  particles(i).history = cell(0);  % an empty cell, will be appended by every old poses
  for l = 1:NUM_LANDMARKS   % initialize the landmarks aka the map
    particles(i).landmarks(l).observed = false;
    particles(i).landmarks(l).mu = zeros(2,1);    % 2D position of the landmark
    particles(i).landmarks(l).sigma = zeros(2,2); % covariance of the landmark
  end
end

% Record each plot as a video %
%video_recorder = VideoWriter('fastSLAM', 'MPEG-4');
%video_recorder.FrameRate = 15;
%open(video_recorder);

% Perform filter update for each odometry-observation pair read from the
% data file.
for t = 1:size(data.timestep, 2)
%for t = 1:50
    fprintf('timestep = %d\n', t);

    % Perform the prediction step of the particle filter
    particles = prediction_step(particles, data.timestep(t).odometry, MOTION_NOISE);

    % Perform the correction step of the particle filter
    particles = correction_step(particles, data.timestep(t).sensor, SENSOR_NOISE);

    % Generate visualization plots of the current state of the filter
    plot_state(particles, landmarks, t, data.timestep(t).sensor);
    
    % Save current figure and append it in video %
    %frame = getframe;
    %writeVideo(video_recorder, frame);

    % Resample the particle set
    particles = resample(particles);
end

% End the video recorder %
%close(video_recorder);
