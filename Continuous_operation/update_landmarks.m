function [S,B] = update_landmarks(S,B,KLT_tracker_C,image,K,hyper_paras)
% extract rotation and translation
R_W_C = reshape(S.est_rot(:,end),[3,3]);
t_W_C = S.est_trans(:,end);
T_W_C = [R_W_C, t_W_C;zeros(1,3), 1];

R_C_W = R_W_C';
t_C_W = -R_W_C'*t_W_C;

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
% S.F_W = S.F_W(:,validity_candidate);
% S.F_W = S.F_W(:,inliersIndex);
S.T = S.T(:,validity_candidate);
S.T = S.T(:,inliersIndex);

% calculate angle
num_candidate = size(matched_points_valid_candidate,1);
p_current_homo = [fliplr(matched_points_valid_candidate)';ones(1,num_candidate)]; % (row, col) to (u,v)
p_current_normalied_homo = K\p_current_homo;

p_first_normalied_homo = zeros(size(p_current_normalied_homo));
for i = 1:num_candidate
    T_W_C_first = [reshape(S.T(:,i),[3,4]);zeros(1,3), 1];
    T_C_Cf = T_W_C\T_W_C_first;
    
    p_first_normalied_homo(:,i) = T_C_Cf(1:3,1:3)*(K\[flipud(S.F(:,i));1]);
end

angles = acos(sum(p_current_normalied_homo.*p_first_normalied_homo,1)./...
    (vecnorm(p_current_normalied_homo).*vecnorm(p_first_normalied_homo)));
angles = angles.*180./pi; % rad to degree
whehter_append = angles>hyper_paras.angle_threshold;

% plot_add_candidate_debug(S.C,fliplr(matched_points_candidate),image,image,validity_candidate,inliersIndex,angles);
% append landmarks for candidate keypoints whose angle is larger
% than given threshold
num_added = sum(whehter_append);
p_first = [flipud(S.F(:,whehter_append)); ones(1,num_added)]; %(u,v,1)
T_vec_first = S.T(:,whehter_append);
p_current = p_current_homo(:,whehter_append); %(u,v,1)
M_current_C_W = K*[R_C_W t_C_W];

num = [0;0;0]; % to count outlier because of other reason

for ii = 1:num_added
    T_W_C_first = [reshape(T_vec_first(:,ii),[3,4]);zeros(1,3), 1];
    T_C_first_W = inv(T_W_C_first);
    M_first_C_W = K*T_C_first_W(1:3,:);

    P_est = linearTriangulation(p_current(:,ii),p_first(:,ii),M_current_C_W,M_first_C_W);
    
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
    reproj1 = M_current_C_W*P_est;
    reproj1 = reproj1/reproj1(3);
    reproj2 = M_first_C_W*P_est;
    reproj2 = reproj2/reproj2(3);
    
    % filter points behind and too far from the camera
    P_est_local_coord = T_W_C\P_est;
    if P_est_local_coord(3) > hyper_paras.min_depth && P_est_local_coord(3) < hyper_paras.max_depth 
        S.X = [S.X, [P_est(1:3); B.new_idx]]; % TODO: if speed up, pay attention to this
        B.landmarks = [B.landmarks, [P_est(1:3); B.new_idx]]; % TODO: if speed up, pay attention to this
        B.new_idx = B.new_idx + 1;
        B.m = size(B.landmarks,2);
        S.P = [S.P, [p_current(2,ii); p_current(1,ii)]]; % (u,v) to (row, col) 
    end
        
%     if P_est(3) > T_W_C(3) && norm(reproj1-p_current(:,ii)) < 40 && norm(reproj2-p_first(:,ii)) < 40
% %         disp("reproject error is:"+[reproj1-p_current(:,ii),reproj2-p_first(:,ii)])
%         S.X = [S.X, [P_est(1:3); B.new_idx]]; % TODO: if speed up, pay attention to this
%         B.landmarks = [B.landmarks, [P_est(1:3); B.new_idx]]; % TODO: if speed up, pay attention to this
%         B.new_idx = B.new_idx + 1;
%         B.m = size(B.landmarks,2);
%         S.P = [S.P, [p_current(2,ii); p_current(1,ii)]]; % (u,v) to (row, col) 
%     else
%         if P_est(3) <= T_W_C(3)
%             num(1) = num(1) + 1;
%         end
%         if norm(reproj1-p_current(:,ii)) > 50
%             num(2) = num(2) + 1;
%         end
%         if norm(reproj2-p_first(:,ii)) > 50
%             num(3) = num(3) + 1;
%         end
%     end
end

% update state
S.C = matched_points_valid_candidate(~whehter_append,:)';
S.F = S.F(:,~whehter_append);
S.T = S.T(:,~whehter_append);
S.num_new = [S.num_new; num_added];

end