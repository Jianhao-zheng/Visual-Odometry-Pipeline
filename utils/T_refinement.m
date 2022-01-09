function refined_T_W_C = T_refinement(T_W_C, keypoints, landmarks, K)
twist_T = HomogMatrix2twist(T_W_C);
error_terms=@(T)reproj_error(T,double(keypoints), double(landmarks), K);  %transfer to double to increase accuracy
options = optimoptions(@lsqnonlin,'Display','None','MaxIter',100);
optimized_twist_T = lsqnonlin(error_terms,twist_T,[],[], options);
refined_T_W_C = twist2HomogMatrix(optimized_twist_T);
end

function [error] = reproj_error(T, keypoints,landmarks, K)
T_W_C = twist2HomogMatrix(T);
T_C_W = inv(T_W_C);

landmarks_homo = [landmarks; ones(1, size(landmarks,2))];
reproj_keypoints = K*T_C_W(1:3,:)*landmarks_homo;
reproj_keypoints = reproj_keypoints./(reproj_keypoints(3,:));

error = keypoints - reproj_keypoints(1:2,:);

error = error(:); 
% error = vecnorm(error); % L2, more reasonable in theory
end
