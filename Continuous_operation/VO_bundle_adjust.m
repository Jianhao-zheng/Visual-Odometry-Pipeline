function [S,B] = VO_bundle_adjust(S,B,T_W_C,K)

if B.count_frame == B.keyframe_d %it's a new key frame (optimize both landmarks and poses)
    if B.num_key < B.window_size
        B.num_key = B.num_key + 1;
        B.tau(1:6,B.num_key) = HomogMatrix2twist(T_W_C);

        % update in 'update_landmarks.m
    %     if B.landmarks(4,end) < S.X(4,end)
    %         B.landmarks = [B.landmarks, S.X(:,find(S.X(4,:)==B.landmarks(4,end))+1:end)];
    %         B.m = size(B.landmarks,2);
    %     end

        Obs_current = [size(S.P,2);S.P(:);S.X(4,:)'];
        B.observation{B.num_key} = Obs_current;
    else
        % discard landmarks stored for BA
        m_new_first_key = B.observation{2}(1);
        lm_idx_new_first = B.observation{2}(end-m_new_first_key+1:end);
        last_landmark = lm_idx_new_first(end);
        last_idx = find(B.landmarks(4,:)==last_landmark);
        landmarks1 = B.landmarks(:,1:last_idx);
        landmarks2 = B.landmarks(:,last_idx+1:end);
        [~,remaining] = find(B.landmarks(4,:)==lm_idx_new_first);
        B.landmarks = [landmarks1(:,remaining), landmarks2];
        
        % update T and Observation
        B.tau = [B.tau(:,2:end), HomogMatrix2twist(T_W_C)];
        Obs_current = [size(S.P,2);S.P(:);S.X(4,:)'];
        B.observation = {B.observation{2:end}, Obs_current};
        B.m = size(B.landmarks,2);
        
        % update landmark index in observation
        observation_loc = B.observation;
        for i = 1:B.num_key
            obs = B.observation{i};
            index_lm_old = obs(end-obs(1)+1:end);
            [~,index_lm_new] = find(B.landmarks(4,:)==index_lm_old);
            obs(end-obs(1)+1:end) = index_lm_new';
            observation_loc{i} = obs;
        end

        state = zeros(6*B.num_key+3*B.m,1);
        state(1:B.num_key*6) = B.tau(:);
        temp = B.landmarks(1:3,:);
        state(B.num_key*6+1:end) = temp(:);
        Obs = [B.num_key;B.m];
        for i = 1:B.num_key
            Obs = [Obs; observation_loc{i}];
        end
        optimized_state = runBA(state, Obs, K);

        for i = 1:B.num_key
            T = twist2HomogMatrix(optimized_state((i-1)*6+1:(i-1)*6+6));
            S.est_rot(:,end-3*(B.num_key-i)) = [T(1:3,1); T(1:3,2); T(1:3,3)];
            S.est_trans(:,end-3*(B.num_key-i)) = T(1:3,4);
        end
        B.landmarks(1:3,:) = reshape(optimized_state(B.num_key*6+1:end),[3,B.m]);

        for i = 1:size(S.X,2)
            idx = find(B.landmarks(4,:)==S.X(4,i)); %%%can be speed up
            S.X(1:3,i) = B.landmarks(1:3,idx);
        end
    end
    B.count_frame = 0;
else % it's a normal frame (optimize poses only)
    if B.num_normal_frame < (B.window_size-1)*B.keyframe_d
        B.num_normal_frame = B.num_normal_frame + 1;
        B.normal_frame_refine{B.num_normal_frame} = {T_W_C,flipud(S.P),S.X(4,:)};
    else
        current_frame_refine = {T_W_C,flipud(S.P),S.X(4,:)};
        B.normal_frame_refine = {B.normal_frame_refine{2:end}, current_frame_refine};
    end
    B.count_frame = B.count_frame + 1;
end