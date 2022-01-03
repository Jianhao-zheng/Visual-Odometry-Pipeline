%% Setup
close all;clear;clc;

% add path of functions
addpath(genpath('utils'))
addpath('Continuous_operation')
addpath('Initialization')

ds = 1; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = 'data/kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = 'data/malaga';
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = 'data/parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);

    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames for each dataset

if ds == 0
    % hint from project statement
    bootstrap_frames = [0 2]; % naming from `000000.png`
    img_seq_len = bootstrap_frames(2) - bootstrap_frames(1);

    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    % img1 = imread([kitti_path '/05/image_0/' ...
    %     sprintf('%06d.png',bootstrap_frames(2))]);

    sz = size(img0);
    img_seqs = ones(sz(1), sz(2), img_seq_len);

    % import all images between bootstrap_frames 
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs(:,:,i) = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png', fr_idx)]);
    end
elseif ds == 1
    bootstrap_frames = [1 3]; % elements from `left_images` list
    img_seq_len = bootstrap_frames(2) - bootstrap_frames(1);

    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    % img1 = rgb2gray(imread([malaga_path ...
    %     '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
    %     left_images(bootstrap_frames(2)).name]));

    sz = size(img0);
    img_seqs = ones(sz(1), sz(2), img_seq_len);

    % import all images between bootstrap_frames 
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs(:,:,i) = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(fr_idx).name]));
    end
elseif ds == 2
    bootstrap_frames = [0 2]; % naming from `img_00000.png`
    img_seq_len = bootstrap_frames(2) - bootstrap_frames(1);

    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    % img1 = rgb2gray(imread([parking_path ...
    %     sprintf('/images/img_%05d.png',bootstrap_frames(2))]));

    sz = size(img0);
    img_seqs = ones(sz(1), sz(2), img_seq_len);

    % import all images between bootstrap_frames 
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs(:,:,i) = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',fr_idx)]));
    end
else
    assert(false);
end

% KLT init
cameraParams = cameraParameters('IntrinsicMatrix', K');
KLT_tracker_init = vision.PointTracker('BlockSize',[15 15],'NumPyramidLevels',2,...
    'MaxIterations',50,'MaxBidirectionalError',3);

landmark_3d = [];
trans_array = [];

[features_0, valid_key_candidates_0] = detectkeypoints(img0);
init_points_ = valid_key_candidates_0.Location; % 10e3*10e2
initialize(KLT_tracker_init, init_points_, img0);

for i = 1:img_seq_len-1
    % todo: make img_seqs directly
    % im2uint8 会转化成全部255，不太对
    img_intermediate = uint8(img_seqs(:,:,i));
    [~,~] = KLT_tracker_init(img_intermediate);
end

img1 = uint8(img_seqs(:,:,img_seq_len));

[matched_points1_, validity_img1] = KLT_tracker_init(img1);
matched_points_valid = fliplr(matched_points1_(validity_img1,:));

init_points = init_points_(validity_img1,:);

% method1: FundamentalMatrix without known camera parameters
% [fRANSAC, inliers] = estimateFundamentalMatrix(flip(init_points),...
% flip(matched_points_valid),'Method','RANSAC',...
% 'NumTrials',2000,'DistanceThreshold',1e-2);
% disp('Estimated inlier ratio from method2 is');
% disp(nnz(inliers)/numel(inliers));
% essMat = K'*fRANSAC*K;

% method2: EssentialMatrix with known camera parameters
[essMat, inliers] = estimateEssentialMatrix(init_points,matched_points_valid,cameraParams);
disp('Estimated inlier ratio from method1 is');
disp(nnz(inliers)/numel(inliers));

% choose inlier points
init_points = init_points(inliers,:);
matched_points_valid = matched_points_valid(inliers,:);

% visualize for detected feature debugging
plot_KLT_debug(...
    fliplr(init_points_)', ...
    fliplr(matched_points1_),...
    img0,...
    img1,...
    validity_img1,...
    inliers);

% estimate relative pose relative to frame previous frame
[ori, loc] = relativeCameraPose(...
    essMat,...
    cameraParams,...
    init_points,...
    matched_points_valid);
% disp('Found transformation T_C_W = ');
% T_C_W = [ori loc'; zeros(1, 3) 1];
% disp(T_C_W);
% rot_mat = ori';
% trans_vec = -ori'*loc';
[rot_mat, trans_vec] = cameraPoseToExtrinsics(ori, loc);

% compute projection matrix
proj_mat0 = compProjMat(eye(3), [0 0 0]', cameraParams.IntrinsicMatrix');
proj_mat1 = compProjMat(rot_mat, trans_vec', cameraParams.IntrinsicMatrix');


% exe6 solution
% to homogeneous pixel coordinate
init_points_homo = [init_points'; ones(1,length(init_points))];
matched_points_valid_homo = [matched_points_valid'; ones(1,length(matched_points_valid))];
% pts3d: samples x 4
pts3d = linearTriangulation(...
    init_points_homo, ...
    matched_points_valid_homo, ...
    proj_mat0, ...
    proj_mat1);
% choose first three dimensions
pts3d = pts3d(1:3,:)';

% filter points according to height (optional)
% consider points lower than `max_height`
consider_height = false; % true, false
if consider_height
    disp(max(pts3d(3,:)))
    disp(min(pts3d(3,:))) % 可能存在 -7.7913 的点
    min_height = -2.0;
    max_height = 30.0;
    valid_idx = (pts3d(:,3) >= min_height) & (pts3d(:,3) <= max_height);
    pts3d = pts3d(valid_idx,:);
    init_points = init_points(valid_idx,:);
    matched_points_valid = matched_points_valid(valid_idx,:);
end
% filter points according to height (optional)

% initial results
p_W_landmarks = pts3d;
keypoints = matched_points_valid;

% debug trangulated points
% figure(111)
% plot3(p_W_landmarks(:,1), p_W_landmarks(:,2), p_W_landmarks(:,3), 'o');
% Display camera pose
% plotCoordinateFrame(R_C_W', t_W_C', 2);
% 这里可视化有点问题，如果分步执行好像没有什么问题，
% 但是不间隔执行以下两步会出现箭头颠倒的情况
% plotCoordinateFrame(eye(3), [0 0 0]', 2);
% text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
% plotCoordinateFrame(ori', trans_vec', 2);
% text(trans_vec(1)-0.1, trans_vec(2)-0.1, trans_vec(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

% patch matching
patch_matching = false;
if patch_matching
    corner_patch_size = 9;
    harris_kappa = 0.08;
    num_keypoints = 200;
    nonmaximum_supression_radius = 8;
    descriptor_radius = 9;
    match_lambda = 4;

    harris_scores0 = harris(img0, corner_patch_size, harris_kappa);
    harris_scores1 = harris(img1, corner_patch_size, harris_kappa);
    keypoints0 = selectKeypoints(...
        harris_scores0, num_keypoints, nonmaximum_supression_radius);
    keypoints1 = selectKeypoints(...
        harris_scores1, num_keypoints, nonmaximum_supression_radius);
    % 361 x 200
    descriptors0 = describeKeypoints(img0, keypoints0, descriptor_radius);
    descriptors1 = describeKeypoints(img1, keypoints1, descriptor_radius);
    %Match descriptors
    matches = matchDescriptors(descriptors1, descriptors0, match_lambda);
    %plotMatches(matches, keypoints1, keypoints0);
    [~, indx1, indx0] = find(matches);
    matchedPoints0 = keypoints0(:,indx0);
    matchedPoints1 = keypoints1(:,indx1);

    figure(2);
    imshow(img1);
    hold on;
    plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);
    plot([matchedPoints0(2,:); matchedPoints1(2,:)], ...
         [matchedPoints0(1,:); matchedPoints1(1,:)], ...
         'g-', 'Linewidth', 3);

    % or directly choose from the following detected Harries points
    % [feat0, valid0] = detectkeypoints(img0);  % 416 x 64
    % [feat1, valid1] = detectkeypoints(img1);  % 422 x 64
    % selectedNumFeatures = min(feat0.NumFeatures, feat1.NumFeatures);
    % selectedDescriptors0 = feat0.Features(1:selectedNumFeatures,:)';
    % selectedDescriptors1 = feat1.Features(1:selectedNumFeatures,:)';
    % matches_new = matchDescriptors(selectedDescriptors1, selectedDescriptors0, match_lambda);
    % newKeypoints0 = valid0.Location(:,[2 1])';
    % newKeypoints1 = valid1.Location(:,[2 1])';
    % %plotMatches(matches_new, newKeypoints1, newKeypoints0);

    % use RANSAC to filter outliers out
    % M-by-2 matrices of M number of [x y] coordinates
    % [~,inliersIndex] = estimateFundamentalMatrix(S.C(:,validity_candidate)',matched_points_valid_candidate);
    [fRANSAC, inliers] = estimateFundamentalMatrix(flip(matchedPoints0)',...
        flip(matchedPoints1)','Method','RANSAC',...
        'NumTrials',2000,'DistanceThreshold',1e-4);

    E = K'*fRANSAC*K;

    matchedPoints0 = matchedPoints0(:,inliersIndex);
    matchedPoints1 = matchedPoints1(:,inliersIndex);

    matchedPoints0 = [matchedPoints0;ones(1,length(matched_points0))];
    matchedPoints1 = [matchedPoints1;ones(1,length(matchedPoints1))];
end



%% Directly get bootstrap from exe7, for debugging continuous operation only
debug = true;

K = load('data/data_exe7/K.txt');
S.P = load('data/data_exe7/keypoints.txt')'; %(row,col)
S.X = load('data/data_exe7/p_W_landmarks.txt')';
S.C = [];%(row,col)
S.F = [];%(row,col)
S.F_W = []; % normalized image coordinates (expressed in world coordinate)
S.T = [];
idx = rand(200,1);

database_image = imread('data/data_exe7/000000.png');
bootstrap_frames = zeros(2,1);
last_frame = 9;

%% Continuous operation

% generate and initialize KLT tracker
% for landmark tracking
KLT_tracker_L = vision.PointTracker('BlockSize',[15 15],'NumPyramidLevels',2,...
    'MaxIterations',50,'MaxBidirectionalError',3);
initialize(KLT_tracker_L,fliplr(S.P'),database_image);
% [features, valid_key_points] = detectkeypoints(database_image);
% initialize(KLT_tracker_c,valid_key_points.Location,database_image);
prev_img = database_image;

% for candidate keypoints tracking
KLT_tracker_C = vision.PointTracker('BlockSize',[15 15],'NumPyramidLevels',2,...
    'MaxIterations',50,'MaxBidirectionalError',3);

% parameters for discarding redundant new candidate keypoints
r_discard_redundant = 5;

% parameters for deciding whether or not to add a triangulated landmark
angle_threshold = pi*5/180; %start with pi*10/180 dervie by Rule of the thumb:

range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end

    %%%% only for debug
    image = imread(['data/data_exe7/' sprintf('%06d.png',i)]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%% assciate keypoints %%%%%%%%%%%%%%%%%%%%%%%
    % detect keypoints
    [features, valid_key_candidates] = detectkeypoints(image);
    % figure(1); imshow(image); hold on;plot(valid_key_points); % plot

    % KLT tracking
    [matched_points,validity] = KLT_tracker_L(image);
    matched_points_valid = fliplr(matched_points(validity,:));

    % perform RANSAC to find best Pose and inliers
    [R_C_W, t_C_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
        ransacLocalization(matched_points_valid', S.X(:,validity), K);
    R_W_C = R_C_W';
    T_W_C = -R_C_W'*t_C_W;

    % plotting
%     plot_KLT_debug(S.P,fliplr(matched_points),prev_img,image,validity,inlier_mask);

    % update KLT_tracker (for landmarks)
    release(KLT_tracker_L);
    S.P = matched_points_valid((inlier_mask)>0,:)';
    S.X = S.X(:,validity);
    S.X = S.X(:,(inlier_mask)>0);
    initialize(KLT_tracker_L,fliplr(S.P'),image);

    % track candidate keypoints
    if ~isempty(S.C)
        [matched_points_candidate,validity_candidate] = KLT_tracker_C(image);
        matched_points_valid_candidate = fliplr(matched_points_candidate(validity_candidate,:)); %(u,v) to (row,col)

        [~,inliersIndex] = estimateFundamentalMatrix(S.C(:,validity_candidate)',matched_points_valid_candidate);
        matched_points_valid_candidate = matched_points_valid_candidate(inliersIndex,:);
        % plotting
%         plot_KLT_debug(S.C,fliplr(matched_points_candidate),prev_img,image,validity_candidate,true(1,sum(validity_candidate)));
        plot_KLT_debug(S.C,fliplr(matched_points_candidate),prev_img,image,validity_candidate,inliersIndex);

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
            S.X = [S.X, P_est(1:3)];
            S.P = [S.P, [p_current(2,ii); p_current(1,ii)]]; % (u,v) to (row, col)
        end

        % update state
        S.C = matched_points_valid_candidate(~whehter_append,:)';
        S.F = S.F(:,~whehter_append);
        S.F_W = S.F_W(:,~whehter_append);
        S.T = S.T(:,~whehter_append);
    end

    % discard redundant new candidate keypoints (whose distance to any
    % existing keypoints is less than 'r_discard_redundant')
    redundant_map = ones(size(image));
    for ii = 1:size(S.P,2)
        redundant_map(max(1,floor(S.P(1,ii)-r_discard_redundant)):...
            min(ceil(S.P(1,ii)+r_discard_redundant),size(redundant_map,1)),...
            max(1,floor(S.P(2,ii)-r_discard_redundant)):...
            min(ceil(S.P(2,ii)+r_discard_redundant),size(redundant_map,2))) = 0;
    end

    for ii = 1:size(S.C,2)
        redundant_map(max(1,floor(S.C(1,ii)-r_discard_redundant)):...
            min(ceil(S.C(1,ii)+r_discard_redundant),size(redundant_map,1)),...
            max(1,floor(S.C(2,ii)-r_discard_redundant)):...
            min(ceil(S.C(2,ii)+r_discard_redundant),size(redundant_map,2))) = 0;
    end

    no_discard = ones(size(valid_key_candidates.Location,1),1);
    for ii = 1:size(valid_key_candidates.Location,1)
        no_discard(ii) = redundant_map(round(valid_key_candidates.Location(ii,2)),round(valid_key_candidates.Location(ii,1)));
    end
    no_discard = logical(no_discard);

    % plot for debugging
%     plot_discard_debug(image,S,valid_key_candidates,no_discard)

    valid_key_candidates = valid_key_candidates(no_discard);
    S.C = [S.C, double(flipud(valid_key_candidates.Location'))];
    S.F = [S.F, double(flipud(valid_key_candidates.Location'))];
    unnormalized_camera_coord = [double(valid_key_candidates.Location');...
        ones(1,size(valid_key_candidates.Location,1))]; % (u,v)
    normalized_camera_coord = K\unnormalized_camera_coord;
    normalized_camera_coord_world = R_W_C*normalized_camera_coord...
        + repmat(T_W_C, [1 size(normalized_camera_coord,2)]);
    S.F_W = [S.F_W normalized_camera_coord_world];
    S.T = [S.T, repmat([R_C_W(:);t_C_W(:)],1,size(valid_key_candidates.Location,1))];

    % update KLT_tracker (for candidate)
    if ~isempty(S.C)
        release(KLT_tracker_C);
        initialize(KLT_tracker_C,fliplr(S.C'),image);
    end

    % Makes sure that plots refresh.
    pause(0.01);

    prev_img = image;
end