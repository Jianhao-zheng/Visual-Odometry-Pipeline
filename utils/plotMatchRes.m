function plotMatchRes(keypoint_database,matched_points,prev_image,curr_image,validity,inlier_mask)

fh = figure;
% https://ch.mathworks.com/matlabcentral/answers/102219-how-do-i-make-a-figure-full-screen-programmatically-in-matlab
% fh.WindowState = 'maximized';
% [left bottom width height]
fh.Position = [10 10 1200 800];

% https://ch.mathworks.com/matlabcentral/fileexchange/27991-tight_subplot-nh-nw-gap-marg_h-marg_w
% use tight_subplot to control subplot spacing
% [ha, pos] = tight_subplot(Nh, Nw, gap, marg_h, marg_w)
[ha, ~] = tight_subplot(2,2,[.02 .05],[.1 .01],[.01 .01]);


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
scatter(matched_points(2,validity), matched_points(1,validity),'r');
title('Matched keypoints in current image');
axis('image');
axis('tight');

% subplot(2, 2, 3);
axes(ha(3)); 
imshow(prev_image)
hold on
scatter(keypoint_database(2,validity),keypoint_database(1,validity),'r');
title('Matched keypoints in previous image');
axis('image');
axis('tight');

% subplot(2, 2, 4);
axes(ha(4)); 
imshow(curr_image)
hold on
corresponding_matches = 1:size(matched_points,2);
matched_query_keypoints = matched_points(:,validity);
keypoints = keypoint_database(:,validity);
plotMatches(corresponding_matches(inlier_mask>0), ...
    matched_query_keypoints(:, inlier_mask>0), ...
    keypoints);
title('Matched keypoints after RANSAC in current image');
axis('image');
axis('tight');

end