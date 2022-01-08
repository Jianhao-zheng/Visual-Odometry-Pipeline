function B = update_bundle_struct(B,S)

B.window_size = 5; %size of window to do bundle adjustment
B.n_frame = 1;
B.tau = [];
B.landmarks = S.X;
B.discard_idx = cell(1,window_size); % buffer recording which landmarks to discard
B.observation = cell(1,window_size);
B.observation{1}.num_key = size(S.P,2);
B.observation{1}.P = S.P; %(row,col)
B.observation{1}.P_idx = S.X(4,:);

% if B.n_frame < B.window_size
    