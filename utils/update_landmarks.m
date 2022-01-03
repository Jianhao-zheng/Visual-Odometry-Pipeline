function [S,B] = update_landmarks(S,B,KLT_tracker_C,image,K,angle_threshold)
% extract rotation and translation
R_W_C = reshape(S.est_rot(:,end),[3,3]);
T_W_C = S.est_trans(:,end);

R_C_W = R_W_C';
t_C_W = -R_W_C'*T_W_C;

[matched_points_candidate,validity_candidate] = KLT_tracker_C(image);
matched_points_valid_candidate = fliplr(matched_points_candidate(validity_candidate,:)); %(u,v) to (row,col)

[~,inliersIndex] = estimateFundamentalMatrix(S.C(:,validity_candidate)',matched_points_valid_candidate,...
    'NumTrials',500,'Method','RANSAC','DistanceThreshold',5e-2);
matched_points_valid_candidate = matched_points_valid_candidate(inliersIndex,:);
% plotting
% plot_KLT_debug(S.C,fliplr(matched_points_candidate),prev_img,image,validity_candidate,true(1,sum(validity_candidate)));
% plot_KLT_debug(S.C,fliplr(matched_points_candidate),prev_img,image,validity_candidate,inliersIndex);

S.F = S.F(:,validity_candidate);
S.F = S.F(:,inliersIndex);
S.F_W = S.F_W(:,validity_candidate);
S.F_W = S.F_W(:,inliersIndex);
S.T = S.T(:,validity_candidate);
S.T = S.T(:,inliersIndex);

% calculate angle
temp = fliplr(matched_points_valid_candidate)'; % (row, col) to (u,v)
normalized_matched_candidate = K\[temp; ones(1,size(temp,2))];
normalized_matched_candidate_world = R_W_C*normalized_matched_candidate...
    + repmat(T_W_C, [1 size(normalized_matched_candidate,2)]);
angles = acos(sum(normalized_matched_candidate_world.*S.F_W,1)./...
    (vecnorm(normalized_matched_candidate_world).*vecnorm(S.F_W)));
whehter_append = angles>angle_threshold;

% append landmarks for candidate keypoints whose angle is larger
% than given threshold
num_added = sum(whehter_append);
p_first = [flipud(S.F(:,whehter_append)); ones(1,num_added)]; %(u,v,1)
M_vec_first = S.T(:,whehter_append);
p_current = [temp(:,whehter_append); ones(1,num_added)]; %(u,v,1)
M_current = K*[R_C_W t_C_W];
for ii = 1:num_added
    M_first = K*[reshape(M_vec_first(1:9,ii),[3,3]) M_vec_first(10:12,ii)]; % can be speeded up for candidate from same first frame
    P_est = linearTriangulation(p_current(:,ii),p_first(:,ii),M_current,M_first);
    S.X = [S.X, [P_est(1:3); S.X(4,end)+ii]]; % TODO: if speed up, pay attention to this
    B.landmarks = [B.landmarks, [P_est(1:3); B.landmarks(4,end)+ii]]; % TODO: if speed up, pay attention to this
    S.P = [S.P, [p_current(2,ii); p_current(1,ii)]]; % (u,v) to (row, col) 
end

% update state
S.C = matched_points_valid_candidate(~whehter_append,:)';
S.F = S.F(:,~whehter_append);
S.F_W = S.F_W(:,~whehter_append);
S.T = S.T(:,~whehter_append);

end