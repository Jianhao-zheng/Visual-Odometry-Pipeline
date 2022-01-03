function proj_mat = compProjMat(rot_mat, trans_vec, intrinsic_mat)
    % rot_mat [3 x 3]
    % trans_vec [3 x 1]
    % IntrinsicMat from `cameraParams.IntrinsicMatrix'`
    proj_mat = intrinsic_mat * [rot_mat, trans_vec];
end