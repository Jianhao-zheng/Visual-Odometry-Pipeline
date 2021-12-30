function plot_KLT_debug(S,matched_points,prev_image,image,validity,inlier_mask)
figure(3)
subplot(2, 2, 1);
imshow(prev_image)
hold on
% scatter(tt(2,:),tt(1,:),'r');
scatter(S.P(2,:),S.P(1,:),'r');
title('All keypoints in previous image');

subplot(2,2,2);
imshow(image)
hold on
t2=matched_points';
scatter(t2(2,validity),t2(1,validity),'r');
title('Matched keypoints in current image');

subplot(2, 2, 3);
imshow(prev_image)
hold on
scatter(S.P(2,validity),S.P(1,validity),'r');
title('Matched keypoints in previous image');

subplot(2, 2, 4);
imshow(image)
hold on
corresponding_matches = 1:size(matched_points,1);
matched_query_keypoints = t2(:,validity);
keypoints = S.P(:,validity);
plotMatches(corresponding_matches(inlier_mask>0), ...
    matched_query_keypoints(:, inlier_mask>0), ...
    keypoints);
title('Matched keypoints after Ransac');
end