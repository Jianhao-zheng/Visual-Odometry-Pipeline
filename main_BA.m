%% Setup
close all;clear;clc;

% add path of functions
addpath(genpath('utils'))
addpath('Continuous_operation')  
addpath('Initialization')  

ds = 2; % 0: KITTI, 1: Malaga, 2: parking

% hyperparameters
feature_extract = 'SURF';

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
else
    assert(false);
end

%% initialization with KLT
cameraParams = cameraParameters('IntrinsicMatrix', K');
KLT_tracker_init = vision.PointTracker(...
    'NumPyramidLevels',5,...
    'MaxBidirectionalError',3);

% [features0, valid_key_candidates0] = detectkeypoints(img0);
[features0, valid_key_candidates0] = genKeypoints(img0,feature_extract);
init_points_ = valid_key_candidates0.Location;
initialize(KLT_tracker_init, init_points_, img0);

for i = 1:img_seq_len-1
    img_intermediate = img_seqs{i};
    [~,~] = KLT_tracker_init(img_intermediate);
end

img1 = img_seqs{end};
[matched_points1_, validity_img1] = KLT_tracker_init(img1);
matched_points_valid = matched_points1_(validity_img1,:);

init_points = init_points_(validity_img1,:);
    
% method1: FundamentalMatrix without known camera parameters
[fRANSAC, inliers] = estimateFundamentalMatrix( ...
    init_points,matched_points_valid, ...
    'Method','RANSAC',...
    'NumTrials',2000, ...
    'DistanceThreshold',1e-2);
disp('Estimated inlier ratio from method2 is');
disp(nnz(inliers)/numel(inliers));
essMat = K'*fRANSAC*K;

% method2: EssentialMatrix with known camera parameters
% [essMat, inliers] = estimateEssentialMatrix(init_points,matched_points_valid,cameraParams,'Confidence', 99.9);
% disp('Estimated inlier ratio from method1 is');
% disp(nnz(inliers)/numel(inliers));

% choose inlier points
init_points = init_points(inliers,:);
matched_points_valid = matched_points_valid(inliers,:);

% visualize for detected feature debugging
plotMatchRes(...
    fliplr(init_points_)', ...
    fliplr(matched_points1_)',...
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
[rot_mat, trans_vec] = cameraPoseToExtrinsics(ori, loc);
% disp(ground_truth(bootstrap_frames(2),:))
% disp(rot_mat)
% disp(trans_vec)

% compute projection matrix
proj_mat0 = compProjMat(eye(3), [0 0 0]', cameraParams.IntrinsicMatrix');
proj_mat1 = compProjMat(rot_mat, trans_vec', cameraParams.IntrinsicMatrix');

% convert to homogeneous pixel coordinate
init_points_homo = [init_points'; ones(1,length(init_points))];
matched_points_valid_homo = [matched_points_valid'; ones(1,length(matched_points_valid))];
% pts3d (samples x 4)
pts3d = linearTriangulation(...
    init_points_homo, ...
    matched_points_valid_homo, ...
    proj_mat0, ...
    proj_mat1);
% back from homogeneous coordinate
pts3d = pts3d(1:3,:)';

% filter points according to height (optional)
% consider points between [min_z, max_z]
consider_z = true; % true, false
if consider_z
    disp(max(pts3d(3,:)))
    disp(min(pts3d(3,:))) % 可能存在 -7.7913 的点
    min_z = 0.0;
    max_z = 100.0;
    valid_idx = (pts3d(:,3) >= min_z) & (pts3d(:,3) <= max_z);
    pts3d = pts3d(valid_idx,:);
    init_points = init_points(valid_idx,:);
    matched_points_valid = matched_points_valid(valid_idx,:);
end

% initial results for continuous operation
p_W_landmarks = double(pts3d);
keypoints = double(matched_points_valid);
T_init_WC = [ori, loc'];
T_init_WC = pose_refinement(T_init_WC, keypoints, p_W_landmarks, K);

% debug trangulated points
debug_trangulation = false;
if debug_trangulation
    figure(111)
    plot3(p_W_landmarks(:,1), p_W_landmarks(:,2), p_W_landmarks(:,3), 'o');
    % display camera pose
    plotCoordinateFrame(eye(3), [0 0 0]', 5);
    text(-0.1,-0.1,-0.1,'Cam 1', ...
        'fontsize',10, ...
        'color','k', ...
        'FontWeight','bold');
    plotCoordinateFrame(ori', trans_vec', 5);
    text(trans_vec(1)-0.1, trans_vec(2)-0.1, trans_vec(3)-0.1,'Cam 2', ...
        'fontsize',10, ...
        'color','k', ...
        'FontWeight','bold');
end

%% Directly get bootstrap from exe7, for debugging continuous operation only
debug = true;

% Directly get bootstrap from exe7
% K = load('data/data_exe7/K.txt');
% S.P = load('data/data_exe7/keypoints.txt')'; %(row,col)
% S.X = load('data/data_exe7/p_W_landmarks.txt')';

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
B.window_size = 5; %size of window to do bundle adjustment
B.n = 1;
B.m = size(S.X,2);
B.tau = HomogMatrix2twist([ori, loc';zeros(1,3) 1]);
B.landmarks = S.X;
B.discard_idx = cell(1,B.window_size); % buffer recording which landmarks to discard
B.observation = cell(1,B.window_size);
% B.observation{1} = O_2;
B.new_idx = B.m + 1; %index when adding new keypoints


% database_image = imread('data/data_exe7/000000.png');
% bootstrap_frames = zeros(2,1);
% last_frame = 9;

switch ds % 0: KITTI, 1: Malaga, 2: parking
    case 0
        database_image = imread([kitti_path '/05/image_0/' ...
                sprintf('%06d.png', 2)]);
        gt_scale = ground_truth./(ground_truth(3,2)/S.est_trans(3,1)); % for kitti
    case 1
        database_image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(fr_idx).name]));
        gt_scale = zeros(600,2);
    case 2
        database_image = rgb2gray(imread([parking_path ...
                sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
        gt_scale = ground_truth./(ground_truth(bootstrap_frames(2)+1,1)/S.est_trans(1,1)); % for praking
end

plot_all(database_image,S,gt_scale,2,bootstrap_frames(2))
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
angle_threshold = 5; %start with 10 degree dervie by Rule of the thumb:
    
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
    
%     %%%% only for debug
%     image = imread(['data/data_exe7/' sprintf('%06d.png',i)]);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% assciate keypoints %%%%%%%%%%%%%%%%%%%%%%%
    % detect keypoints
    [features, valid_key_candidates] = genKeypoints(img0,feature_extract); 
    % figure(1); imshow(image); hold on;plot(valid_key_points); % plot
    
    % KLT tracking
    [matched_points,validity] = KLT_tracker_L(image);
    matched_points_valid = fliplr(matched_points(validity,:)); % (u,v) to (row,col)
    
    % perform RANSAC to find best Pose and inliers
    [R_C_W, t_C_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
        ransacLocalization(matched_points_valid', S.X(1:3,validity), K);
    T_C_W = [R_C_W, t_C_W];
    
    R_W_C = R_C_W';
    t_W_C = -R_C_W'*t_C_W;
    T_W_C = [R_W_C, t_W_C];
    
    % plotting
%     plot_KLT_debug(S.P,fliplr(matched_points),prev_img,image,validity,inlier_mask);
    
    % discard unmatched landmarks
    S.P = matched_points_valid((inlier_mask)>0,:)';
    S.X = S.X(:,validity);
    S.X = S.X(:,(inlier_mask)>0);
    
    T_W_C = pose_refinement(T_W_C, fliplr(S.P'), S.X(1:3,:)', K);
    
    S.est_trans = [S.est_trans, T_W_C(:,4)];
    R_W_C = T_W_C(:,1:3);
    S.est_rot = [S.est_rot, R_W_C(:)];
    
    % track candidate keypoints
    if ~isempty(S.C)
        [S,B] = update_landmarks(S,B,KLT_tracker_C,image,K,angle_threshold);
    end
%     [S,B] = VO_bundle_adjust(S,B,M_W_C,K);
    S = update_candidate(S,valid_key_candidates,image,K,r_discard_redundant);
    
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

    plot_all(image,S,gt_scale,2,i)

    % Makes sure that plots refresh.    
    pause(0.1);
    
    prev_img = image;
end