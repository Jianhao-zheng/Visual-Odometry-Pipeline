function [pts3d,matched_points_valid] = triangulation_init(init_points_valid,matched_points_valid,T_init_WC,K,hyper_paras)
T_C_W = inv(T_init_WC);
R_C_W = T_C_W(1:3,1:3);
t_C_W = T_C_W(1:3,4);

% compute projection matrix
proj_mat0 = compProjMat(eye(3), [0; 0; 0], K);
proj_mat1 = compProjMat(R_C_W, t_C_W, K);

% convert to homogeneous pixel coordinate
init_points_valid_homo = [init_points_valid'; ones(1,length(init_points_valid))];
matched_points_valid_homo = [matched_points_valid'; ones(1,length(matched_points_valid))];
% pts3d (samples x 4)
pts3d = linearTriangulation(...
    init_points_valid_homo, ...
    matched_points_valid_homo, ...
    proj_mat0, ...
    proj_mat1);
% back from homogeneous coordinate
pts3d = pts3d(1:3,:)';

% filter points according to distance (optional)
% consider points between [min_z, max_z]
consider_depth = true; % true, false
if consider_depth
%     disp(max(pts3d(3,:)))
%     disp(min(pts3d(3,:))) 
    valid_idx = (pts3d(:,3) >= hyper_paras.min_depth) & (pts3d(:,3) <= hyper_paras.max_depth);
    pts3d = pts3d(valid_idx,:);
%     init_points = init_points(valid_idx,:);
    matched_points_valid = matched_points_valid(valid_idx,:);
end

end