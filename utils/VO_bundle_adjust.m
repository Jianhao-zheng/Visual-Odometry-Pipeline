function [S,B] = VO_bundle_adjust(S,B,M_W_C,K)
if B.n < B.window_size
    B.n = B.n + 1;
    B.tau(1:6,B.n) = HomogMatrix2twist([M_W_C;zeros(1,3) 1]);
    
    % update in 'update_landmarks.m
%     if B.landmarks(4,end) < S.X(4,end)
%         B.landmarks = [B.landmarks, S.X(:,find(S.X(4,:)==B.landmarks(4,end))+1:end)];
%         B.m = size(B.landmarks,2);
%     end

    Obs_current = [size(S.P,2);S.P(:);S.X(4,:)'];
    B.observation{B.n} = Obs_current;
else
    B.tau = [B.tau(:,2:end), HomogMatrix2twist([M_W_C;zeros(1,3) 1])];
    Obs_current = [size(S.P,2);S.P(:);S.X(4,:)'];
    B.observation = {B.observation{2:end}, Obs_current};
end

state = zeros(6*B.n+3*B.m,1);
state(1:B.n*6) = B.tau(:);
temp = B.landmarks(1:3,:);
state(B.n*6+1:end) = temp(:);
Obs = [B.n;B.m];
for i = 1:B.n
    Obs = [Obs; B.observation{i}];
end
optimized_state = runBA(state, Obs, K);

for i = 1:B.n
    T = twist2HomogMatrix(optimized_state((i-1)*6+1:(i-1)*6+6));
    S.est_rot(:,i) = [T(1:3,1); T(1:3,2); T(1:3,3)];
    S.est_trans(:,i) = T(1:3,4);
end
B.landmarks(1:3,:) = reshape(optimized_state(B.n*6+1:end),[3,B.m]);

for i = 1:size(S.X,2)
    idx = find(B.landmarks(4,:)==S.X(4,i)); %%%can be speed up
    S.X(1:3,i) = B.landmarks(1:3,idx);
end
end