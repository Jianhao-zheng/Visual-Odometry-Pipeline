%% Setup
close all;
clear;
clc;

% add path of functions
addpath(genpath('utils'))
addpath('Continuous_operation')
addpath('Initialization')

% 0: KITTI
% 1: Malaga
% 2: parking
% 3: epfl_parking (customized)
% 4: lausanne_center_nav (customized)
ds = 0;
datasets = {'kitti', 'malaga', 'parking', 'epfl_parking', 'lausanne_center_nav'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% hyperparameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% whether to use BA (do not activate here, please use main_BA.m if you want to test BA)
hyper_paras.is_BA = false; 

hyper_paras.is_refine_pose = true; % whether to refine pose estimation by reprojection error

hyper_paras.feature_extract = 'SURF'; %method to extract features
hyper_paras.feature_extract_options = {'MetricThreshold', 10};
% hyper_paras.feature_extract = 'Harris';
% hyper_paras.feature_extract_options = {'MinQuality',1e-6};

hyper_paras.init_matching_method = 'KLT'; % method to matching keypoints, two options: ['KLT', 'Des_match']
hyper_paras.show_matching_res = false;
hyper_paras.sfm_pose = 'fundamental'; % method to estimate pose from 2D-2D, two options: ['fundamental', 'essential']

% range of vaild landmarks (filter out points behind and too far from the
% camera)
hyper_paras.min_depth = 0;
hyper_paras.max_depth = 100;

% parameters for discarding redundant new candidate keypoints
hyper_paras.r_discard_redundant = 5; % [pixel]

% parameters for deciding whether or not to add a triangulated landmark
hyper_paras.angle_threshold = 5; %start with 10 degree dervie by Rule of the thumb:

% use visualization
hyper_paras.viz_all = true; % true false

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_tic = tic;

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = 'data/kitti';
    assert(exist('kitti_path', 'var') ~= 0);

    % last_frame = 2760;
    last_frame = 540; % 540 20
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];

    % tuned hyperparameters
    if hyper_paras.is_BA
        hyper_paras.feature_extract_options = {'MetricThreshold', 1000}; %less keypoint to speed up
%         hyper_paras.feature_extract_options = {'MinQuality',1e-6}; %less keypoint to speed up
    else
        hyper_paras.feature_extract_options = {'MetricThreshold', 200};
%         hyper_paras.feature_extract_options = {'MinQuality',1e-4};
    end

    % specific hyperparameters
    hyper_paras.min_depth = 1;
    hyper_paras.r_discard_redundant = 10;
    hyper_paras.max_depth = 80;
    hyper_paras.angle_threshold = 1;

    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    has_gt = true;
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

    % specific hyperparameters
    hyper_paras.feature_extract_options = {'MetricThreshold', 400};
    hyper_paras.min_depth = 2;
    hyper_paras.r_discard_redundant = 10;
    hyper_paras.max_depth = 50;
    hyper_paras.angle_threshold = 0.8;

    has_gt = false;
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = 'data/parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);

    % specific hyperparameters
    hyper_paras.feature_extract_options = {'MetricThreshold', 100};
    hyper_paras.min_depth = 2;
    hyper_paras.r_discard_redundant = 10;
    hyper_paras.max_depth = 50;
    hyper_paras.angle_threshold = 0.8;

    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    has_gt = true;
elseif ds == 3
    % Path containing images, depths and all...
    epfl_path = 'data/epfl_parking';
    assert(exist('epfl_path', 'var') ~= 0);
    image_dir = dir([epfl_path '/image']);
    images = image_dir(3:end);
    % last_frame = length(images);
    last_frame = 170; % large motion blur after them
    K = load([epfl_path '/calibration' '/K.txt']);
    % no GT available for now!!!

    % specific hyperparameters
    hyper_paras.feature_extract_options = {'MetricThreshold', 500};
    hyper_paras.min_depth = 2;
    hyper_paras.r_discard_redundant = 7;
    hyper_paras.max_depth = 50;
    hyper_paras.angle_threshold = 0.8;

    has_gt = false;
elseif ds == 4
    % Path containing images, depths and all...
    lausanne_path = 'data/lausanne_center_nav';
    assert(exist('lausanne_path', 'var') ~= 0);
    image_dir = dir([lausanne_path '/image']);
    images = image_dir(3:end);
    last_frame = 136;
    K = load([lausanne_path '/K.txt']);
    % no GT available for now!!!

    % specific hyperparameters
    hyper_paras.feature_extract_options = {'MetricThreshold', 10};
    hyper_paras.min_depth = 2;
    hyper_paras.r_discard_redundant = 7;
    hyper_paras.max_depth = 50;
    hyper_paras.angle_threshold = 0.8;

    has_gt = false;
else
    assert(false);
end

toc_param = toc(param_tic);

%% Bootstrap
% need to set bootstrap_frames for each dataset
bootstrap_tic = tic;
if ds == 0
    % hint from project statement
    bootstrap_frames = [0 2]; % naming from `000000.png`
    img_seq_len = bootstrap_frames(2) - bootstrap_frames(1);

    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    % img1 = imread([kitti_path '/05/image_0/' ...
    %     sprintf('%06d.png',bootstrap_frames(2))]);

    img_seqs = cell(img_seq_len,1);
    % import intermediate images between bootstrap_frames
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs{i} = imread([kitti_path '/05/image_0/' ...
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

    img_seqs = cell(img_seq_len,1);
    % import intermediate images between bootstrap_frames
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs{i} = rgb2gray(imread([malaga_path ...
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

    img_seqs = cell(img_seq_len,1);
    % import intermediate images between bootstrap_frames
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs{i} = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',fr_idx)]));
    end
elseif ds == 3
    bootstrap_frames = [1 3]; % naming from `0000.png`
    img_seq_len = bootstrap_frames(2) - bootstrap_frames(1);

    img0 = rgb2gray(imread([epfl_path ...
        '/image/' ...
        images(bootstrap_frames(1)).name]));

    img_seqs = cell(img_seq_len,1);
    % import intermediate images between bootstrap_frames
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs{i} = rgb2gray(imread([epfl_path ...
        '/image/' ...
        images(fr_idx).name]));
    end
elseif ds == 4
    bootstrap_frames = [0 2]; % naming from `0000.png`
    img_seq_len = bootstrap_frames(2) - bootstrap_frames(1);

    img0 = rgb2gray(imread([lausanne_path ...
        sprintf('/image/%04d.png',bootstrap_frames(1))]));

    img_seqs = cell(img_seq_len,1);
    % import intermediate images between bootstrap_frames
    for i = 1:img_seq_len
        fr_idx = i + bootstrap_frames(1);
        img_seqs{i} = rgb2gray(imread([lausanne_path ...
        sprintf('/image/%04d.png',fr_idx)]));
    end
else
    assert(false);
end

%% Initialization
cameraParams = cameraParameters('IntrinsicMatrix', K');

[init_points,matched_points] = matching_init(img0,img_seqs,hyper_paras);

[T_init_WC,init_points_valid,matched_points_valid] = pose_estimation_init(init_points,matched_points,K,hyper_paras);

[pts3d,matched_points_valid] = triangulation_init(init_points_valid,matched_points_valid,T_init_WC,K,hyper_paras);

% initial results for continuous operation
p_W_landmarks = double(pts3d);
keypoints = double(matched_points_valid);

if hyper_paras.is_refine_pose
    T_init_WC = T_refinement(T_init_WC, keypoints', p_W_landmarks', K);
end

toc_bootstrap = toc(bootstrap_tic);

%% Initialize state

ct_tic = tic;

S.X = p_W_landmarks';
S.P = double(fliplr(keypoints))';

S.X = [S.X; 1:size(S.X,2)]; % add a row to indicate the landmark index (for BA)
S.C = [];%(row,col)
S.F = [];%(row,col)
% S.F_W = []; % vectors pointing from optical center to normalized image coordinates (expressed in world coordinate)
S.T = [];
S.est_trans = T_init_WC(1:3,4); % estimated camera translation (3 x N)
temp = T_init_WC(1:3,1:3);
S.est_rot = temp(:);% estimated camera rotation (9 x N)

S.num_X = size(S.X,2);
S.num_C = size(S.C,2);
S.num_new = 0;

% struct for bundle adjustment
B.window_size = 3; %size of window to do bundle adjustment (# of keyframes)
B.keyframe_d = 2; % we choose keyframe every (keyframe_d+1) frame
B.num_key = 1; % number of keyframes stored
B.count_frame = 0; % auxiliary variable to decide whether next is a key frame, taking value from [0,..,B.keyframe_d]
B.m = size(S.X,2);
B.tau = HomogMatrix2twist(T_init_WC);
B.landmarks = S.X;
B.observation = cell(1,B.window_size);
temp = flipud(S.P);
B.observation{1} = [B.m;temp(:);(1:B.m)'];

B.normal_frame_refine = cell(1,(B.window_size-1)*B.keyframe_d);
B.num_normal_frame = 0;

B.new_idx = B.m + 1; %index when adding new keypoints


% database_image = imread('data/data_exe7/000000.png');
% bootstrap_frames = zeros(2,1);
% last_frame = 9;

switch ds % 0: KITTI, 1: Malaga, 2: parking
    case 0
        gt_scale = ground_truth./(ground_truth(3,2)/S.est_trans(3,1)); % for kitti
    case 1
        gt_scale = zeros(600,2);
    case 2
        gt_scale = ground_truth./(ground_truth(bootstrap_frames(2)+1,1)/S.est_trans(1,1)); % for praking
    case 3
        % for epfl_parking
        gt_scale = zeros(last_frame+bootstrap_frames(2),2);
    case 4
        % for lausanne_center_nav
        gt_scale = zeros(last_frame+bootstrap_frames(2),2);
end

database_image = img_seqs{end};
if hyper_paras.viz_all
    plot_frame_stat(...
        database_image,...
        S,...
        gt_scale,...
        2,...
        bootstrap_frames(2),...
        hyper_paras.max_depth,...
        has_gt...
        )
end

%% Continuous operation

% generate and initialize KLT tracker
% for landmark tracking
KLT_tracker_L = vision.PointTracker('BlockSize',[21 21],'NumPyramidLevels',5,...
    'MaxIterations',20,'MaxBidirectionalError',6);
initialize(KLT_tracker_L,fliplr(S.P'),database_image);
prev_img = database_image;

% for candidate keypoints tracking
KLT_tracker_C = vision.PointTracker('BlockSize',[21 21],'NumPyramidLevels',5,...
    'MaxIterations',20,'MaxBidirectionalError',6);

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
    elseif ds == 3
        image = rgb2gray(imread([epfl_path ...
            '/image/' ...
            images(i).name]));
    elseif ds == 4
        image = im2uint8(rgb2gray(imread([lausanne_path ...
            sprintf('/image/%04d.png',i)])));
    else
        assert(false);
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%% associate keypoints %%%%%%%%%%%%%%%%%%%%%%%
    % detect keypoints
    [features, valid_key_candidates] = genKeypoints(image,hyper_paras.feature_extract,hyper_paras.feature_extract_options);

    % KLT tracking
    [matched_points,validity] = KLT_tracker_L(image);
    matched_points_valid = fliplr(matched_points(validity,:)); % (u,v) to (row,col)

    disp(length(validity))
    disp(sum(validity) / length(validity))

    % perform RANSAC to find best Pose and inliers
    [R_C_W, t_C_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
        ransacLocalization(matched_points_valid', S.X(1:3,validity), K);
    T_C_W = [R_C_W, t_C_W; zeros(1,3), 1];

    T_W_C = inv(T_C_W);

    % discard unmatched landmarks
    S.P = matched_points_valid((inlier_mask)>0,:)';
    S.X = S.X(:,validity);
    S.X = S.X(:,(inlier_mask)>0);

    if hyper_paras.is_refine_pose
        T_W_C = T_refinement(T_W_C, flipud(S.P), S.X(1:3,:), K);
    end
    S.est_trans = [S.est_trans, T_W_C(1:3,4)];
    R_W_C = T_W_C(1:3,1:3);
    S.est_rot = [S.est_rot, R_W_C(:)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Adding new keypoints and candidates %%%%%%%%%%%%%%%%%%%%%%%
    % track candidate keypoints
    if ~isempty(S.C)
        [S,B] = update_landmarks(S,B,KLT_tracker_C,image,K,hyper_paras);
    end
    if hyper_paras.is_BA
        [S,B] = VO_bundle_adjust(S,B,T_W_C,K);
    end

    S = update_candidate(S,valid_key_candidates,image,K,hyper_paras);


    % update KLT_tracker (for landmarks)
    release(KLT_tracker_L);
    initialize(KLT_tracker_L,fliplr(S.P'),image);

    % update KLT_tracker (for candidate)
    if ~isempty(S.C)
        release(KLT_tracker_C);
        initialize(KLT_tracker_C,fliplr(S.C'),image);
    end

    S.num_X = [S.num_X; size(S.X,2)];
    S.num_C = [S.num_C; size(S.C,2)];

    if hyper_paras.viz_all
        plot_frame_stat(...
            image,...
            S,...
            gt_scale,...
            2,...
            i,...
            hyper_paras.max_depth,...
            has_gt...
            )
        % Makes sure that plots refresh.
        pause(0.01);
    end

    prev_img = image;
end

toc_ct = toc(ct_tic);
frame_init = img_seq_len+1;
frame_ct = numel(range);
fps_ct = frame_ct/toc_ct;

total_time = toc_param + toc_bootstrap + toc_ct;

% TIC, TOC, errs  autosave
eval_path = './eval';
dataset_str = ['dataset@' datasets{ds+1}];
feat_str = ['feat@' hyper_paras.feature_extract];
now_str = char(datetime('now','TimeZone','local','Format','HH_mm_ss'));
time_str = ['time@' now_str];
experiment_name = strcat(dataset_str, '_', feat_str, '_', time_str);
if ~hyper_paras.is_refine_pose
    experiment_name = strcat(experiment_name, '_NoRefined');
end
save_name = [eval_path '/' experiment_name];

if has_gt
    errs = quantitative_eval(...
        ground_truth,...
        S,...
        bootstrap_frames,...
        save_name);
end

save([save_name, '_res.mat'], ...
    'errs', 'S', ...
    'total_time', 'frame_init', 'frame_ct', ...
    "toc_param", "toc_bootstrap","toc_ct", "fps_ct")
