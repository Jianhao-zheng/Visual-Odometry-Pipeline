function errs = quantitative_eval(ground_truth,S,bootstrap_frames, save_name)

save_fig = true;

GT = ground_truth(bootstrap_frames(2)+1:bootstrap_frames(2)+size(S.est_trans,2),:)';
GT = [GT(1,:); zeros(1,size(GT,2));GT(2,:)];
Estimated = S.est_trans;

distance_between_frames = vecnorm(GT(:,2:end) - GT(:,1:end-1));

dist_checkpoints = [10;40;90;160;250;360];
dist_checkpoints = dist_checkpoints.*0.2; % for parking only
errs = [];

current_dist = 0;
next_dist_checkpoint = dist_checkpoints(1);
dist_checkpoints = dist_checkpoints(2:end);


for i = 1:size(GT,2)-1
    current_dist = current_dist + distance_between_frames(i);
    if current_dist > next_dist_checkpoint
        aligned_est = alignEstimateToGroundTruth(...
            GT(:,1:i), Estimated(:,1:i));
        replacement_gt = GT(:,i)-GT(:,1);
        replacement_est = aligned_est(:,end)-aligned_est(:,1);
        errs = [errs, abs(replacement_est-replacement_gt)];

        figure(3)
%         plot(GT(3, 1:i), -GT(1, 1:i),'LineWidth',5);
%         hold on;
%         plot(Estimated(3, 1:i), -Estimated(1, 1:i),'LineWidth',5);
%         plot(aligned_est(3, :), -aligned_est(1, :),'LineWidth',5);
%         hold off;
%         axis equal;
%         % axis([-5 95 -30 10]);
%         legend('Ground truth', 'Original estimate', 'Aligned estimate', ...
%             'Location', 'SouthWest','FontSize',25);
%         set(gca,'FontSize',25);
        
        plot(GT(3, 1:i), -GT(1, 1:i),'LineWidth',5);
        hold on;
%         plot(Estimated(3, 1:i), -Estimated(1, 1:i),'LineWidth',5);
        plot(aligned_est(3, :), -aligned_est(1, :),'LineWidth',5);
        hold off;
        axis equal;
        % axis([-5 95 -30 10]);
        legend('Ground truth',  'Aligned estimate', ...
            'Location', 'SouthWest','FontSize',25);
        set(gca,'FontSize',25);


        if length(dist_checkpoints) == 1
            next_dist_checkpoint = dist_checkpoints(1);
            dist_checkpoints = [];
        elseif isempty(dist_checkpoints)
            break
        else
            next_dist_checkpoint = dist_checkpoints(1);
            dist_checkpoints = dist_checkpoints(2:end);
        end
    end
end

if save_fig
%     print(save_name,'-dpng');
    % print(save_name,'-dpdf','-bestfit');
end

end
