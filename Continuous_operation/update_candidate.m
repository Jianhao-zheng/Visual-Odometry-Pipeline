function S = update_candidate(S,valid_key_candidates,image,K,hyper_paras)
% extract rotation and translation
R_W_C = reshape(S.est_rot(:,end),[3,3]);
t_W_C = S.est_trans(:,end);

R_C_W = R_W_C';
t_C_W = -R_W_C'*t_W_C;

% discard redundant new candidate keypoints (whose distance to any
% existing keypoints is less than 'r_discard_redundant')
redundant_map = ones(size(image)); 
r = hyper_paras.r_discard_redundant;
for ii = 1:size(S.P,2)
    redundant_map(max(1,floor(S.P(1,ii)-r)):...
        min(ceil(S.P(1,ii)+r),size(redundant_map,1)),...
        max(1,floor(S.P(2,ii)-r)):...
        min(ceil(S.P(2,ii)+r),size(redundant_map,2))) = 0;
end

for ii = 1:size(S.C,2)
    redundant_map(max(1,floor(S.C(1,ii)-r)):...
        min(ceil(S.C(1,ii)+r),size(redundant_map,1)),...
        max(1,floor(S.C(2,ii)-r)):...
        min(ceil(S.C(2,ii)+r),size(redundant_map,2))) = 0;
end

no_discard = ones(size(valid_key_candidates.Location,1),1);
for ii = 1:size(valid_key_candidates.Location,1)
    no_discard(ii) = redundant_map(round(valid_key_candidates.Location(ii,2)),round(valid_key_candidates.Location(ii,1)));
end
no_discard = logical(no_discard);

% plot for debugging
% plot_discard_debug(image,S,valid_key_candidates,no_discard)

valid_key_candidates = valid_key_candidates(no_discard); 
S.C = [S.C, double(flipud(valid_key_candidates.Location'))];
S.F = [S.F, double(flipud(valid_key_candidates.Location'))];
% unnormalized_camera_coord = [double(valid_key_candidates.Location');...
%     ones(1,size(valid_key_candidates.Location,1))]; % (u,v)
% normalized_camera_coord = K\unnormalized_camera_coord;
% normalized_camera_coord_world = R_W_C*normalized_camera_coord...
%     + repmat(t_W_C, [1 size(normalized_camera_coord,2)]);
% S.F_W = [S.F_W normalized_camera_coord_world-t_W_C];
S.T = [S.T, repmat([R_W_C(:);t_W_C(:)],1,size(valid_key_candidates.Location,1))];

end