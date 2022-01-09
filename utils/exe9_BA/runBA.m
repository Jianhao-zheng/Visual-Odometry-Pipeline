% modified from original `runBA.m`
function hidden_state = runBA(hidden_state, observations, K)
% Update the hidden state, encoded as explained in the problem statement,
% with 20 bundle adjustment iterations.

with_pattern = true;

if with_pattern
    num_frames = observations(1);
    num_observations = (numel(observations)-2-num_frames)/3;
    % Factor 2, one error for each x and y direction.
    num_error_terms = 2 * num_observations;
    % Each error term will depend on one pose (6 entries) and one landmark
    % position (3 entries), so 9 nonzero entries per error term:
    pattern = spalloc(num_error_terms, numel(hidden_state), ...
        num_error_terms * 9);

    % Fill pattern for each frame individually:
    observation_i = 3;  % iterator into serialized observations
    error_i = 1;  % iterating frames, need another iterator for the error
    for frame_i = 1:num_frames
        num_keypoints_in_frame = observations(observation_i);
        % All errors of a frame are affected by its pose.
        pattern(error_i:error_i+2*num_keypoints_in_frame-1, ...
            (frame_i-1)*6+1:frame_i*6) = 1;

        % Each error is then also affected by the corresponding landmark.
        landmark_indices = observations(...
            observation_i+2*num_keypoints_in_frame+1:...
            observation_i+3*num_keypoints_in_frame);
        for kp_i = 1:numel(landmark_indices)
            pattern(error_i+(kp_i-1)*2:error_i+kp_i*2-1,...
                1+num_frames*6+(landmark_indices(kp_i)-1)*3:...
                num_frames*6+landmark_indices(kp_i)*3) = 1;
        end

        observation_i = observation_i + 1 + 3*num_keypoints_in_frame;
        error_i = error_i + 2 * num_keypoints_in_frame;
    end
%     figure(4);
%     spy(pattern);

    % don't optimize landmarks coordinate if it's not matched for all key
    % frames
    for i = observations(1)*6+1:size(pattern,2)
        if length(find(pattern(:,i)>0)) < 2*2
            pattern(:,i) = 0;
        end
    end

%     figure(5)
%     spy(pattern);
end

% Also here, using an external error function for clean code.
error_terms = @(hidden_state) baError(hidden_state, observations, K);
% options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
%     'MaxIter', 20, 'FunctionTolerance', 1e-2);
% if with_pattern
%     options.JacobPattern = pattern;
%     options.UseParallel = false;
% end
% modidied
tic
options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
    'MaxIter', 1, 'FunctionTolerance', 1e-2);

if with_pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end
state_before_opt = hidden_state;
hidden_state = lsqnonlin(error_terms, hidden_state, [], [], options);
time=toc;

if time < 0.5
    options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
        'MaxIter', 20);

    if with_pattern
        options.JacobPattern = pattern;
        options.UseParallel = false;
    end
    hidden_state = lsqnonlin(error_terms, hidden_state, [], [], options);
else
    hidden_state = state_before_opt;
end
end

