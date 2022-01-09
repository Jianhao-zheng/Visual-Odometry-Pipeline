function plot_frame_stat(image, S, gt, fig_idx, i, max_depth, has_gt)

if nargin < 6
    max_depth = 10;
    has_gt = false;
end

figure(fig_idx);
set(gcf,'outerposition',get(0,'screensize'));
% set(0,'DefaultFigureWindowStyle','docked')

%% plot1: matching result with image
subplot(2, 4, [1 2]);
imshow(image); %, 'InitialMagnification', 800
hold on
if ~isempty(S.C)
    scatter(S.C(2,:),S.C(1,:),20,'gx');
    hold on
end
scatter(S.P(2,:),S.P(1,:),20,'ro');
hold off
if ~isempty(S.C)
    legend('candidate kpts','existing kpts','FontSize',8);
end
title('Current frame: No.'+string(i));

%% plot2: matching result line chart
subplot(2, 4, 5);
plot(S.num_X,'g')
hold on;
plot(S.num_C,'r')
hold on;
plot(S.num_new,'b')
hold off;
legend('# of kpts','# of candidates','# of new kpts','FontSize',8);
title('Current frame: No.'+string(i));

%% plot3: trajectory line chart
subplot(2, 4, 6);

p1 = plot(S.est_trans(1,:),S.est_trans(3,:),'r','MarkerSize',3);
p1.Marker = '*';
hold on;
if has_gt
    p2 = plot(gt(1:i+1,1),gt(1:i+1,2),'b','MarkerSize',3);
    p2.Marker = '*';
    legend('Estimated pose','Scaled GT','FontSize',8, 'Location', 'southwest');
end
% title('Estimated trajectory');
title('Estimated trajectory');
axis equal
hold off;
% axis([min([S.est_trans(1,:),gt(1:i+1,1)']) - 0.25*abs(min([S.est_trans(1,:),gt(1:i+1,1)']))...
%       max([S.est_trans(1,:),gt(1:i+1,1)']) + 0.25*abs(max([S.est_trans(1,:),gt(1:i+1,1)']))...
%       -5 5]) %to be changed

%% plot4: landmarks tracking over last 20 frames
subplot(2, 4, [3 4 7 8]);
scatter(S.X(1,:),S.X(3,:),50,'bx');
hold on;
if size(S.est_trans,2) < 20
    % scatter(S.est_trans(1,:),S.est_trans(3,:),50,'r.');
    p2 = plot(S.est_trans(1,:),S.est_trans(3,:),'r','MarkerSize',10);
    p2.Marker = '.';
    % axis([-30 30 -2 80])
    hold off
else
    % scatter(S.est_trans(1,end-19:end),S.est_trans(3,end-19:end),50,'r.');
    p2 = plot(S.est_trans(1,end-19:end),S.est_trans(3,end-19:end),'r','MarkerSize',10);
    p2.Marker = '.';
    p3 = plot(S.est_trans(1,1:end-19),S.est_trans(3,1:end-19),'Color',[0.5,0.5,0.5]);

    % curr_fr = size(S.est_trans,2);
    fr_win = 15;
    local_sz = 0.5 * max_depth + 10;
    half_local_sz = local_sz * 0.5;
    depth_sz = [-half_local_sz half_local_sz];
    width_sz = [-half_local_sz half_local_sz];
    axis_data = [...
        S.est_trans(1,end) ...
        S.est_trans(1,end) ...
        S.est_trans(3,end) ...
        S.est_trans(3,end) ...
        ];
    change_xy_ratio = true;
    x_scale = S.est_trans(1,end) - S.est_trans(1,end-fr_win+1);
    y_scale = S.est_trans(3,end) - S.est_trans(3,end-fr_win+1);
    if change_xy_ratio
        if abs(x_scale) > abs(y_scale)
            axis_data(3:4) = axis_data(3:4) + width_sz;
            if sign(x_scale) >= 0
                axis_data(1:2) = axis_data(1:2) + depth_sz;
            else
                axis_data(1:2) = axis_data(1:2) + fliplr(-depth_sz);
            end
            xlim(axis_data(1:2))
            ylim(axis_data(3:4))
        else
            axis_data(1:2) = axis_data(1:2) + width_sz;
            if sign(x_scale) >= 0
                axis_data(3:4) = axis_data(3:4) + depth_sz;
            else
                axis_data(3:4) = axis_data(3:4) + fliplr(-depth_sz);
            end
            xlim(axis_data(1:2))
            ylim(axis_data(3:4))
        end
    else
        axis_data(1:2) = axis_data(1:2) + width_sz;
        if sign(x_scale) >= 0
            axis_data(3:4) = axis_data(3:4) + fliplr(-depth_sz);
        else
            axis_data(3:4) = axis_data(3:4) + depth_sz;
        end
    end

    % axis(axis_data)
    hold off

end

legend('landmarks','trajectory','FontSize',12);
title('Trajectory of last 20 frames');