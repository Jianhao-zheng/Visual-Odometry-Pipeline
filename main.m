%% Setup
close all;clear;clc;

% add path of functions
addpath(genpath('utils'))
addpath('Continuous_operation')  
addpath('Initialization')  

ds = 0; % 0: KITTI, 1: Malaga, 2: parking

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
% need to set bootstrap_frames
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end


%% Directly get bootstrap from exe7, for debugging continuous operation only
debug = true;

K = load('data/data_exe7/K.txt');
S.P = load('data/data_exe7/keypoints.txt')';
S.X = load('data/data_exe7/p_W_landmarks.txt')';
idx = rand(200,1);

database_image = imread('data/data_exe7/000000.png');
bootstrap_frames = zeros(2,1);
last_frame = 9;

%% Continuous operation

% generate and initialize KLT tracker
KLT_tracker_c = vision.PointTracker('BlockSize',[15 15],'NumPyramidLevels',2,...
    'MaxIterations',50,'MaxBidirectionalError',3);
initialize(KLT_tracker_c,fliplr(S.P'),database_image);
% [features, valid_key_points] = detectkeypoints(database_image); 
% initialize(KLT_tracker_c,valid_key_points.Location,database_image);
prev_img = database_image;
    
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
    %detect keypoints
%     [features, valid_key_points] = detectkeypoints(image); 
    %figure(1); imshow(image); hold on;plot(valid_key_points); % plot
    
    %KLT tracking
    [matched_points,validity] = KLT_tracker_c(image);
    matched_points_valid = fliplr(matched_points(validity,:));
    
    % perform RANSAC to find best Pose and inliers
    [R_C_W, t_C_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
        ransacLocalization(matched_points_valid', S.X(:,validity), K);
    
    plot_KLT_debug(S,fliplr(matched_points),prev_img,image,validity,inlier_mask);
    
    release(KLT_tracker_c);
    S.P = matched_points_valid((inlier_mask)>0,:)';
    S.X = S.X(:,validity);
    S.X = S.X(:,(inlier_mask)>0);
    initialize(KLT_tracker_c,fliplr(S.P'),image);
    
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
end