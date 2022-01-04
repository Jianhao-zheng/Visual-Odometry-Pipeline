function [S,B] = update_landmarks(S,B,KLT_tracker_C,image,K,angle_threshold)
% extract rotation and translation
R_W_C = reshape(S.est_rot(:,end),[3,3]);
T_W_C = S.est_trans(:,end);

R_C_W = R_W_C';
t_C_W = -R_W_C'*T_W_C;

[matched_points_candidate,validity_candidate] = KLT_tracker_C(image);
matched_points_valid_candidate = fliplr(matched_points_candidate(validity_candidate,:)); %(u,v) to (row,col)

[~,inliersIndex] = estimateFundamentalMatrix(S.C(:,validity_candidate)',matched_points_valid_candidate,...
    'NumTrials',500,'Method','RANSAC','DistanceThreshold',5e-2);
matched_points_valid_candidate = matched_points_valid_candidate(inliersIndex,:);
% plotting
% plot_KLT_debug(S.C,fliplr(matched_points_candidate),prev_img,image,validity_candidate,true(1,sum(validity_candidate)));
% plot_KLT_debug(S.C,fliplr(matched_points_candidate),prev_img,image,validity_candidate,inliersIndex);

S.F = S.F(:,validity_candidate);
S.F = S.F(:,inliersIndex);
S.F_W = S.F_W(:,validity_candidate);
S.F_W = S.F_W(:,inliersIndex);
S.T = S.T(:,validity_candidate);
S.T = S.T(:,inliersIndex);

% calculate angle
temp = fliplr(matched_points_valid_candidate)'; % (row, col) to (u,v)
normalized_matched_candidate = K\[temp; ones(1,size(temp,2))];
normalized_matched_candidate_world = R_W_C*normalized_matched_candidate...
    + repmat(T_W_C, [1 size(normalized_matched_candidate,2)]);
vector_3d = normalized_matched_candidate_world - T_W_C;
angles = acos(sum(vector_3d.*S.F_W,1)./...
    (vecnorm(vector_3d).*vecnorm(S.F_W)));
whehter_append = angles>angle_threshold;

% plot_add_candidate_debug(S.C,fliplr(matched_points_candidate),image,image,validity_candidate,inliersIndex,angles);
% append landmarks for candidate keypoints whose angle is larger
% than given threshold
num_added = sum(whehter_append);
p_first = [flipud(S.F(:,whehter_append)); ones(1,num_added)]; %(u,v,1)
P_first = S.F_W(:,whehter_append); %dir vectors in first seen frame
T_vec_first = S.T(:,whehter_append);
p_current = [temp(:,whehter_append); ones(1,num_added)]; %(u,v,1)
P_current = vector_3d(:,whehter_append); %dir vectors in current frame
angle_append = angles(:,whehter_append);
M_current = K*[R_C_W t_C_W];
num = [0;0;0]; % to count outlier because of other reason
for ii = 1:num_added
    M_first = K*[reshape(T_vec_first(1:9,ii),[3,3]) T_vec_first(10:12,ii)]; % can be speeded up for candidate from same first frame
    P_est = linearTriangulation(p_current(:,ii),p_first(:,ii),M_current,M_first);
    
%     % ???
%     P_est=linearTriangulation_refinement(P_est,p_current(:,ii),p_first(:,ii),M_current,M_first);
    
    R_first = reshape(T_vec_first(1:9,ii),[3,3]);
    T_first = T_vec_first(10:12,ii);
    R_first_W_C = R_first';
    T_first_W_C = -R_first'*T_first;
    
    % inspired by exe5
%     SE3_c_f = [R_C_W t_C_W; zeros(1,3) 1]*[R_first_W_C T_first_W_C; zeros(1,3) 1];
%     P_est = triangulation_2(p_first(:,ii),p_current(:,ii),SE3_c_f,[R_first_W_C T_first_W_C; zeros(1,3) 1],K);
    % for debug
    reproj1 = M_current*P_est;
    reproj1 = reproj1/reproj1(3);
    reproj2 = M_first*P_est;
    reproj2 = reproj2/reproj2(3);
%     if P_est(3) <= T_W_C(3)
%         P_temp = [R_C_W t_C_W; zeros(1,3) 1]*P_est;
%         P_temp(1:3) = -P_temp(1:3);
%         P_est = [R_W_C T_W_C; zeros(1,3) 1]*P_temp;
%     end
        
    if P_est(3) > T_W_C(3) && norm(reproj1-p_current(:,ii)) < 30 && norm(reproj2-p_first(:,ii)) < 30
        disp("reproject error is:"+[reproj1-p_current(:,ii),reproj2-p_first(:,ii)])
        S.X = [S.X, [P_est(1:3); S.X(4,end)+ii]]; % TODO: if speed up, pay attention to this
        B.landmarks = [B.landmarks, [P_est(1:3); B.landmarks(4,end)+ii]]; % TODO: if speed up, pay attention to this
        S.P = [S.P, [p_current(2,ii); p_current(1,ii)]]; % (u,v) to (row, col) 
    else
        if P_est(3) <= T_W_C(3)
            num(1) = num(1) + 1;
        end
        if norm(reproj1-p_current(:,ii)) > 30
            num(2) = num(2) + 1;
        end
        if norm(reproj2-p_first(:,ii)) > 30
            num(3) = num(3) + 1;
        end
    end
end

% update state
S.C = matched_points_valid_candidate(~whehter_append,:)';
S.F = S.F(:,~whehter_append);
S.F_W = S.F_W(:,~whehter_append);
S.T = S.T(:,~whehter_append);
S.num_new = [S.num_new; num_added];

end