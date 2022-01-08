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
if ~isempty(S.C)
    legend('candidate keypoints','existing keypoints','FontSize',12);
end
title('Current frame: No.'+string(i));

subplot(2, 4, 5); 
plot(S.num_X,'g')
hold on; 
plot(S.num_C,'r')
hold on; 
plot(S.num_new,'b')
hold off; 
legend('# of keypoints','# of candidates','# of new key','FontSize',8);
title('Current frame: No.'+string(i));

subplot(2, 4, 6); 
p1 = plot(gt(1:i+1,1),gt(1:i+1,2),'b');
p1.Marker = '*';
hold on; 
p2 = plot(S.est_trans(1,:),S.est_trans(3,:),'r');
p2.Marker = '*';
hold off; 
legend('ground truth','estimated','FontSize',10);
title('Full trajectory');
% axis([min([S.est_trans(1,:),gt(1:i+1,1)']) - 0.25*abs(min([S.est_trans(1,:),gt(1:i+1,1)']))...
%       max([S.est_trans(1,:),gt(1:i+1,1)']) + 0.25*abs(max([S.est_trans(1,:),gt(1:i+1,1)']))...
%       -5 5]) %to be changed

subplot(2, 4, [3 4 7 8]); 
scatter(S.X(1,:),S.X(3,:),50,'bx');
hold on; 
if size(S.est_trans,2)< 20
    scatter(S.est_trans(1,:),S.est_trans(3,:),50,'r.');
%     axis([-30 30 -2 80])
else
    scatter(S.est_trans(1,end-19:end),S.est_trans(3,end-19:end),50,'r.');
%     axis([min(S.est_trans(1,end-19:end))-30 max(S.est_trans(1,end-19:end))+30 -2 80])
hold off; 
legend('landmarks','trajectory','FontSize',16);
title('Trajectory of last 20 frame');


end