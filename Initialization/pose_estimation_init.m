function [T_W_C,init_points_valid,matched_points_valid] = pose_estimation_init(init_points,matched_points,K,hyper_paras)
cameraParams = cameraParameters('IntrinsicMatrix', K');

switch hyper_paras.sfm_pose
    case 'fundamental'
        % method1: FundamentalMatrix without known camera parameters
        [fRANSAC, inliers] = estimateFundamentalMatrix( ...
            init_points,matched_points, ...
            'Method','RANSAC',...
            'NumTrials',2000, ...
            'DistanceThreshold',1e-2, ...
            'Confidence', 99.9);
        disp('Estimated inlier ratio from `estimateFundamentalMatrix` is');
        disp(nnz(inliers)/numel(inliers));
        essMat = K'*fRANSAC*K;
    
    case 'essential'
        % method2: EssentialMatrix with known camera parameters
        [essMat, inliers] = estimateEssentialMatrix(init_points,matched_points,cameraParams,'Confidence', 99.9);
        disp('Estimated inlier ratio from `estimateEssentialMatrix` is');
        disp(nnz(inliers)/numel(inliers));
end

% choose inlier points
init_points_valid = init_points(inliers,:);
matched_points_valid = matched_points(inliers,:);

% estimate relative pose relative to frame previous frame
[R_W_C, t_W_C] = relativeCameraPose(...
    essMat,...
    cameraParams,...
    init_points_valid,...
    matched_points_valid);

T_W_C = [R_W_C, t_W_C'; zeros(1,3), 1];

end