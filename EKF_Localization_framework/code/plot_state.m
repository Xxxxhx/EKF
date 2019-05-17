function plot_state(mu, sigma, landmarks, timestep)
    % Visualizes the state of the extended Kalman filter.
    %
    % The resulting plot displays the following information:
    % - landmark positions
    % - estimate of the filter state

    clf
    hold on
    grid("on")
    L = struct2cell(landmarks);
    figure(1);
    drawprobellipse(mu, sigma, 0.6, 'r');
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'o');
    drawrobot(mu, 'r', 3, 0.3, 0.3);
    xlim([-1, 11])
    ylim([-1, 11])
    drawnow;
    hold off
end
