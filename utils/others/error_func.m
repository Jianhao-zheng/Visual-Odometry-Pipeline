function [error] = error_func(twist, keypoints,landmarks, K)
T_WC = twist2HomogMatrix(twist);
W_landmarks_hom = [landmarks, ones(size(landmarks,1),1)]';
landmark_C_frame = T_WC \ W_landmarks_hom;
xp = landmark_C_frame(1,:) ./ landmark_C_frame(3,:);
yp = landmark_C_frame(2,:) ./ landmark_C_frame(3,:);
new_keypoints = K * [xp; yp; ones(1, length(yp))];
new_keypoints = new_keypoints(1:2, :);
diff = keypoints' - new_keypoints;
error = sqrt(sum(diff.^2, 1)');
end
