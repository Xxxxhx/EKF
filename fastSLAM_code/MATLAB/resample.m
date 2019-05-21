% resample the set of particles.
% A particle has a probability proportional to its weight to get
% selected. A good option for such a resampling method is the so-called low
% variance sampling
function newParticles = resample(particles)
% TODO: resample particles by their important factors
    numParticles = length(particles);
    newParticles = struct('weight',[], 'pose', [], 'history', [], 'landmarks', []);
    
    w = [particles.weight];
    
    % normalize the weight
    w = w / sum(w);
    
    % the cummulative sum
    cs = cumsum(w);
    weightSum = cs(length(cs));
    
    % initialize the step and the current position on the roulette wheel
    step = weightSum / numParticles;
    position = unifrnd(0, weightSum);
    idx = 1;
    
    % walk along the wheel to select the particles
    for i = 1:numParticles
      position = position + step;
      if (position > weightSum)
        position = position - weightSum;
        idx = 1;
      end
      while (position > cs(idx))
        idx=idx+1;
      end
      newParticles(i) = particles(idx);
    end


end
