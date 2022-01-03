function plot_all(image,S,gt,fig_idx,i)
figure(fig_idx)
set(gcf,'outerposition',get(0,'screensize'));
subplot(2, 4, [1 2]); 
imshow(image); %, 'InitialMagnification', 800
hold on;
if ~isempty(S.C)
    scatter(S.C(2,:),S.C(1,:),'gx');
    hold on; 
end
scatter(S.P(2,:),S.P(1,:),'ro');
hold off; 
legend('candidate keypoints','existing keypoints','FontSize',12);
title('Current frame: No.'+string(i));

subplot(2, 4, 5); 
scatter3(S.X(1,:),S.X(2,:),S.X(3,:),'gx');
% hold on; 
% scatter(S.P(2,:),S.P(1,:),'ro');
% legend('candidate keypoints','existing keypoints','FontSize',12);
title('Current frame: No.'+string(i));

subplot(2, 4, 6); 
scatter(gt(1:i+1,1),gt(1:i+1,2),'b.');
hold on; 
scatter(S.est_trans(1,:),S.est_trans(3,:),'r.');
hold off; 
legend('ground truth','estimated','FontSize',10);
title('Full trajectory');

subplot(2, 4, [3 4 7 8]); 
scatter(S.X(1,:),S.X(3,:),'gx');
hold on; 
scatter(S.est_trans(1,:),S.est_trans(3,:),'r.');
hold off; 
legend('landmarks','trajectory','FontSize',16);
title('Trajectory of last 20 frame');
axis([-40 40 0 80])

end