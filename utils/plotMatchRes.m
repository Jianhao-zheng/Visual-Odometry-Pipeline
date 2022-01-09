function plotMatchRes(keypoint_database, matched_points, prev_image, curr_image, validity,inlier_mask)

% validity, inlier_mask
if nargin <= 4
    use_validity = false;
    use_inlier = false;
elseif nargin == 5
    use_validity = true;
    use_inlier = false;
else
    use_validity = true;
    use_inlier = true;
end

fh = figure;
% https://ch.mathworks.com/matlabcentral/answers/102219-how-do-i-make-a-figure-full-screen-programmatically-in-matlab
% fh.WindowState = 'maximized';
% [left bottom width height]
fh.Position = [10 10 1200 800];

% https://ch.mathworks.com/matlabcentral/fileexchange/27991-tight_subplot-nh-nw-gap-marg_h-marg_w
% use tight_subplot to control subplot spacing
% [ha, pos] = tight_subplot(Nh, Nw, gap, marg_h, marg_w)
[ha, ~] = tight_subplot(2,2,[.02 .05],[.05 .05],[.05 .05]);


%subplot(2, 2, 1);
axes(ha(1));
imshow(prev_image)
hold on
scatter(keypoint_database(2,:),keypoint_database(1,:),'r');
title('All keypoints in previous image');
axis('image');
axis('tight');

% subplot(2,2,2);
axes(ha(2));
imshow(curr_image)
hold on
if use_validity
    matched_points2_ = matched_points(2,validity);
    matched_points1_ = matched_points(1,validity);
else
    matched_points2_ = matched_points(2,:);
    matched_points1_ = matched_points(1,:);
end
scatter(matched_points2_, matched_points1_,'r');
title('Matched keypoints in current image');
axis('image');
axis('tight');

% subplot(2, 2, 3);
axes(ha(3));
imshow(prev_image)
hold on
if use_validity
    keypoint_database2_ = keypoint_database(2,validity);
    keypoint_database1_ = keypoint_database(1,validity);
else
    keypoint_database2_ = keypoint_database(2,:);
    keypoint_database1_ = keypoint_database(1,:);
end
scatter(keypoint_database2_,keypoint_database1_,'r');
title('Matched keypoints in previous image');
axis('image');
axis('tight');

% subplot(2, 2, 4);
axes(ha(4));
imshow(curr_image)
hold on
if use_validity
    matched_query_keypoints = matched_points(:,validity);
    corresponding_matches = 1:size(matched_query_keypoints,2);
    keypoints = keypoint_database(:,validity);
    if use_inlier
        plotMatches(corresponding_matches(inlier_mask>0), ...
            matched_query_keypoints(:, inlier_mask>0), ...
            keypoints);
    else
        plotMatches(corresponding_matches, ...
            matched_query_keypoints, ...
            keypoints); 
    end
else
    corresponding_matches = 1:size(matched_points,2);
    plotMatches(corresponding_matches, ...
        matched_points, ...
        keypoint_database);
end

title('Matched keypoints after RANSAC in current image');
axis('image');
axis('tight');

end