function plot_discard_debug(image,S,valid_key_candidates,no_discard)
figure(2)
subplot(2, 2, 1);
imshow(image); hold on;plot(valid_key_candidates);hold on; scatter(S.P(2,:),S.P(1,:),'r');
title('New kepoints before discarding (Red:keypoints; Green:Candidates)');

subplot(2,2,2);
temp = valid_key_candidates(no_discard);
imshow(image)
hold on
plot(temp);
hold on 
temp  = valid_key_candidates(not(no_discard));
plot(temp.Location(:,1),temp.Location(:,2), 'yx');
title('New keypoints (Yellow: discarded; Green: kept');

subplot(2, 2, 3);
temp = valid_key_candidates(no_discard);
imshow(image)
hold on
plot(temp);hold on; scatter(S.P(2,:),S.P(1,:),'r');
title('New keypoints after discarding redundant (Red:keypoints; Green:Candidates)');

subplot(2, 2, 4);
temp = valid_key_candidates(no_discard);
imshow(image)
hold on; scatter(S.P(2,:),S.P(1,:),'r');
hold on 
temp  = valid_key_candidates(not(no_discard));
plot(temp.Location(:,1),temp.Location(:,2), 'yx');
title('Yellow: discarded new points; Red: existing keypoints');
end